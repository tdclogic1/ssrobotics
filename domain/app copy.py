from flask import Flask, render_template, request, jsonify
from flask_cors import CORS
from datetime import datetime

app = Flask(__name__, template_folder='.')
CORS(app, resources={
    r"/check-domains": {
        "origins": ["http://127.0.0.1:5500", "http://localhost:5500"],
        "methods": ["POST", "OPTIONS"],
        "allow_headers": ["Content-Type", "Accept"]
    }
})

@app.route('/')
def index():
    return render_template('domainchecker.html')

@app.route('/check-domains', methods=['POST', 'OPTIONS'])
def check_domains():
    # Handle OPTIONS request for CORS preflight
    if request.method == 'OPTIONS':
        response = jsonify({'status': 'ok'})
        response.headers.add('Access-Control-Allow-Origin', '*')
        response.headers.add('Access-Control-Allow-Headers', 'Content-Type,Accept')
        response.headers.add('Access-Control-Allow-Methods', 'POST')
        return response

    try:
        data = request.get_json()
        if not data or 'domains' not in data:
            return jsonify({'error': 'No domains provided'}), 400

        domains = data['domains']
        print(f"Checking domains: {domains}")  # Debug print
        
        # Create test results
        results = []
        for domain in domains:
            result = {
                'domain': domain,
                'checked_at': datetime.now().isoformat(),
                'status': 'Active Website',
                'dns_exists': True,
                'whois': {
                    'registered': True,
                    'creation_date': '2023-01-01',
                    'expiration_date': '2024-01-01'
                },
                'website': {
                    'active': True,
                    'status_code': 200,
                    'protocol': 'https'
                }
            }
            results.append(result)

        response_data = {
            'check_date': datetime.now().isoformat(),
            'total_domains': len(results),
            'results': results
        }

        return jsonify(response_data)
    except Exception as e:
        print(f"Error occurred: {str(e)}")  # Debug print
        return jsonify({'error': str(e)}), 500

if __name__ == '__main__':
    print("Server starting on http://localhost:5000")
    app.run(debug=True)