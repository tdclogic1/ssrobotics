from flask import Flask, render_template, request, jsonify
from flask_cors import CORS
from datetime import datetime
import json
import os
import requests
import socket
import dns.resolver
import whois
from urllib.parse import urlparse

app = Flask(__name__, template_folder='.')
CORS(app, resources={r"/*": {"origins": ["http://127.0.0.1:5500", "http://localhost:5500"]}})

def save_to_json_file(domain_data, is_active):
    """Save domain data to appropriate JSON file based on active status"""
    filename = 'json/domain_active.json' if is_active else 'json/domain_available.json'
    
    # Create the initial structure if file doesn't exist
    if not os.path.exists(filename):
        initial_data = {
            "last_updated": datetime.now().isoformat(),
            "domains": []
        }
        with open(filename, 'w') as f:
            json.dump(initial_data, f, indent=2)
    
    # Read existing data
    try:
        with open(filename, 'r') as f:
            existing_data = json.load(f)
    except json.JSONDecodeError:
        existing_data = {"domains": []}
    
    # Update the data
    domain_entry = {
        "check_date": datetime.now().isoformat(),
        "domain_data": domain_data
    }
    
    # Check if domain already exists and update it, or append new entry
    domain_name = domain_data['domain']
    domain_exists = False
    for idx, entry in enumerate(existing_data['domains']):
        if entry['domain_data']['domain'] == domain_name:
            existing_data['domains'][idx] = domain_entry
            domain_exists = True
            break
    
    if not domain_exists:
        existing_data['domains'].append(domain_entry)
    
    # Update last_updated timestamp
    existing_data['last_updated'] = [datetime.now().isoformat()]
    
    # Save updated data
    with open(filename, 'w') as f:
        json.dump(existing_data, f, indent=2)

def check_whois(domain):
    """Check domain registration using WHOIS"""
    try:
        w = whois.whois(domain)
        return {
            'registered': bool(w.domain_name),
            'creation_date': str(w.creation_date[0] if isinstance(w.creation_date, list) 
                              else w.creation_date) if w.creation_date else None,
            'expiration_date': str(w.expiration_date[0] if isinstance(w.expiration_date, list)
                              else w.expiration_date) if w.expiration_date else None
        }
    except Exception as e:
        print(f"WHOIS error for {domain}: {str(e)}")
        return {
            'registered': None,
            'creation_date': None,
            'expiration_date': None
        }

def check_dns_with_fallback(domain):
    """Check DNS with multiple resolvers and better error handling"""
    # List of public DNS resolvers to try
    public_resolvers = [
        '8.8.8.8',        # Google DNS
        '1.1.1.1',        # Cloudflare DNS
        '9.9.9.9',        # Quad9
        '208.67.222.222'  # OpenDNS
    ]
    
    for resolver_ip in public_resolvers:
        try:
            resolver = dns.resolver.Resolver()
            resolver.nameservers = [resolver_ip]
            resolver.timeout = 2
            resolver.lifetime = 2
            
            try:
                answers = resolver.resolve(domain, 'A')
                if answers:
                    return True, None
            except dns.resolver.NXDOMAIN:
                return False, "Domain does not exist"
            except dns.resolver.NoAnswer:
                continue
            except dns.resolver.NoNameservers:
                continue
            except dns.resolver.Timeout:
                continue
        except Exception as e:
            continue
            
    return False, "Could not resolve domain"

def check_domain_availability(domain):
    """Perform actual checks on domain availability and accessibility"""
    result = {
        'domain': domain,
        'checked_at': datetime.now().isoformat(),
        'dns_exists': False,
        'status': 'Unknown',
        'whois': {
            'registered': False,
            'creation_date': None,
            'expiration_date': None
        },
        'website': {
            'active': False,
            'status_code': None,
            'protocol': None
        }
    }

    try:
        # Check WHOIS first
        whois_result = check_whois(domain)
        result['whois'] = whois_result
        
        if whois_result['registered']:
            result['status'] = 'Registered'
        else:
            result['status'] = 'Available'

        # Check DNS with improved error handling
        dns_exists, dns_error = check_dns_with_fallback(domain)
        result['dns_exists'] = dns_exists
        
        if not dns_exists:
            if whois_result['registered']:
                result['status'] = 'Registered (DNS Issue)'
            else:
                result['status'] = 'Available'
            return result

        # Check website accessibility
        for protocol in ['https', 'http']:
            try:
                url = f"{protocol}://{domain}"
                response = requests.get(url, timeout=5, allow_redirects=True)
                if response.status_code == 200:
                    result['website']['active'] = True
                    result['website']['status_code'] = response.status_code
                    result['website']['protocol'] = protocol
                    result['status'] = 'Active Website'
                    break
                else:
                    result['website']['status_code'] = response.status_code
            except requests.RequestException:
                continue

        # Update final status based on all checks
        if whois_result['registered']:
            if result['website']['active']:
                result['status'] = 'Active Website'
            else:
                result['status'] = 'Registered (No Active Website)'
        else:
            if result['dns_exists'] or result['website']['active']:
                result['status'] = 'Active (Registration Data Unavailable)'
            else:
                result['status'] = 'Available'

    except Exception as e:
        print(f"Error checking domain {domain}: {str(e)}")
        result['status'] = f'Error: {str(e)}'

    return result

@app.route('/')
def index():
    return render_template('domainchecker.html')

@app.route('/load-domains-json', methods=['GET', 'OPTIONS'])
def load_domains_json():
    if request.method == 'OPTIONS':
        return jsonify({'status': 'ok'})

    try:
        current_dir = os.path.dirname(os.path.abspath(__file__))
        json_path = os.path.join(current_dir, 'json/domains.json')
        
        if not os.path.exists(json_path):
            return jsonify({
                'error': 'json/domains.json file not found',
                'searched_path': json_path
            }), 404
            
        with open(json_path, 'r') as f:
            data = json.load(f)
            return jsonify(data)
            
    except Exception as e:
        return jsonify({
            'error': 'Server error',
            'message': str(e)
        }), 500

@app.route('/check-domains', methods=['POST', 'OPTIONS'])
def check_domains():
    if request.method == 'OPTIONS':
        return jsonify({'status': 'ok'})

    try:
        data = request.get_json()
        if not data or 'domains' not in data:
            return jsonify({'error': 'No domains provided'}), 400

        domains = data['domains']
        print(f"Checking domains: {domains}")
        
        results = []
        for domain in domains:
            # Clean the domain input
            domain = domain.strip().lower()
            if not domain:
                continue
                
            # Remove any protocol prefixes if present
            domain = urlparse(domain).netloc or domain
            
            # Perform actual domain check
            result = check_domain_availability(domain)
            
            # Determine if domain is active based on both registration and website status
            is_active = result['whois']['registered'] or result['website']['active']
            save_to_json_file(result, is_active)
            
            results.append(result)

        response_data = {
            'check_date': datetime.now().isoformat(),
            'total_domains': len(results),
            'results': results
        }

        return jsonify(response_data)
        
    except Exception as e:
        print(f"Error occurred: {str(e)}")
        return jsonify({'error': str(e)}), 500

if __name__ == '__main__':
    print("Server starting on http://localhost:5000")
    app.run(debug=True)