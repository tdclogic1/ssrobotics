from flask import Flask, render_template, request, jsonify
from flask_cors import CORS
import json
import whois
import dns.resolver
import requests
from datetime import datetime
import traceback

app = Flask(__name__)
CORS(app)  # Enable CORS for all routes

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/check-domains', methods=['POST'])  # Explicitly specify POST method
def check_domains():
    if request.method != 'POST':
        return jsonify({'error': 'Only POST method is allowed'}), 405
        
    try:
        # Get request data
        data = request.get_json()
        if not data or 'domains' not in data:
            return jsonify({'error': 'No domains provided'}), 400

        domains = data['domains']
        if not isinstance(domains, list):
            return jsonify({'error': 'Domains must be provided as a list'}), 400

        results = []
        for domain in domains:
            try:
                # Check DNS
                try:
                    dns.resolver.resolve(domain, 'A')
                    dns_exists = True
                except (dns.resolver.NXDOMAIN, dns.resolver.NoAnswer):
                    dns_exists = False
                except Exception:
                    dns_exists = None

                # Check WHOIS
                try:
                    w = whois.whois(domain)
                    whois_info = {
                        'registered': bool(w.domain_name),
                        'creation_date': str(w.creation_date[0] if isinstance(w.creation_date, list) 
                                          else w.creation_date) if w.creation_date else None,
                        'expiration_date': str(w.expiration_date[0] if isinstance(w.expiration_date, list)
                                          else w.expiration_date) if w.expiration_date else None
                    }
                except Exception:
                    whois_info = {'registered': None, 'creation_date': None, 'expiration_date': None}

                # Check website
                website_info = {'active': False, 'status_code': None, 'protocol': None}
                for protocol in ['https://', 'http://']:
                    try:
                        response = requests.get(f"{protocol}{domain}", timeout=5)
                        website_info = {
                            'active': True,
                            'status_code': response.status_code,
                            'protocol': protocol.replace('://', '')
                        }
                        break
                    except requests.RequestException:
                        continue

                # Determine status
                if website_info['active']:
                    status = 'Active Website'
                elif whois_info['registered']:
                    status = 'Registered (No Active Website)'
                elif dns_exists:
                    status = 'Has DNS (Status Unclear)'
                elif whois_info['registered'] is None:
                    status = 'Check Failed'
                else:
                    status = 'Possibly Available'

                results.append({
                    'domain': domain,
                    'checked_at': datetime.now().isoformat(),
                    'status': status,
                    'dns_exists': dns_exists,
                    'whois': whois_info,
                    'website': website_info
                })

            except Exception as e:
                print(f"Error checking domain {domain}: {str(e)}")
                results.append({
                    'domain': domain,
                    'checked_at': datetime.now().isoformat(),
                    'status': 'Error',
                    'error': str(e)
                })

        response_data = {
            'check_date': datetime.now().isoformat(),
            'total_domains': len(results),
            'results': results
        }

        return jsonify(response_data)

    except Exception as e:
        print("Error occurred:", str(e))
        print(traceback.format_exc())
        return jsonify({
            'error': 'Server error occurred',
            'details': str(e)
        }), 500

if __name__ == '__main__':
    app.run(debug=True)