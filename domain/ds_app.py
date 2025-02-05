from flask import Flask, render_template, request, jsonify
from flask_cors import CORS
from datetime import datetime
import json
import os

# Get the current directory
current_dir = os.path.dirname(os.path.abspath(__file__))

# Initialize Flask app with the correct template folder
app = Flask(__name__, 
            template_folder=os.path.join(current_dir, 'templates'))
CORS(app, resources={r"/*": {"origins": ["http://127.0.0.1:5500", "http://localhost:5500", "http://localhost:5000"]}})

def ensure_json_files_exist():
    """Create JSON files if they don't exist"""
    files = ['domain_active.json', 'domain_available.json']
    initial_data = {
        "last_updated": datetime.now().isoformat(),
        "domains": []
    }
    
    for filename in files:
        file_path = os.path.join(current_dir, filename)
        if not os.path.exists(file_path):
            with open(file_path, 'w') as f:
                json.dump(initial_data, f, indent=2)
            print(f"Created {filename}")

@app.route('/')
def index():
    try:
        return render_template('domain_search.html')
    except Exception as e:
        print(f"Template error: {str(e)}")
        print(f"Current directory: {current_dir}")
        print(f"Template folder: {app.template_folder}")
        return f"Error loading template: {str(e)}", 500

@app.route('/search')
def search_page():
    return render_template('domain_search.html')

@app.route('/search-domains')
def search_domains():
    try:
        # Get search parameters
        filename = request.args.get('file', 'domain_active.json')
        search_term = request.args.get('search', '').lower()

        # Validate filename
        if filename not in ['domain_active.json', 'domain_available.json']:
            return jsonify({'error': 'Invalid filename'}), 400

        # Ensure files exist
        ensure_json_files_exist()

        # Read the JSON file
        file_path = os.path.join(current_dir, filename)
        with open(file_path, 'r') as f:
            data = json.load(f)

        # Filter domains based on search term
        if search_term:
            filtered_domains = [
                entry for entry in data['domains']
                if search_term in entry['domain_data']['domain'].lower()
            ]
        else:
            filtered_domains = data['domains']

        # Sort results by check date (most recent first)
        filtered_domains.sort(key=lambda x: x['check_date'], reverse=True)

        return jsonify({
            'last_updated': data.get('last_updated', datetime.now().isoformat()),
            'domains': filtered_domains
        })

    except Exception as e:
        print(f"Search error: {str(e)}")
        return jsonify({
            'error': 'Search failed',
            'message': str(e)
        }), 500

if __name__ == '__main__':
    print(f"Starting Domain Search Application...")
    print(f"Current directory: {current_dir}")
    print(f"Template folder: {os.path.join(current_dir, 'templates')}")
    ensure_json_files_exist()
    app.run(debug=True, port=5000)