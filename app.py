#from openai.api_resources.completion import Completion
from flask import Flask, render_template, request, jsonify
from joblib import dump, load
import requests
import sklearn
import psycopg2
import json
import os

""" This is a flask app that serves API endpoints for inference """

print(os.environ)

app = Flask(__name__)

'''
t0pp() takes no arguments but asks t0pp to translate text to SQL queries
Example: https://www.dystopiarobotics.com/t0pp?prompt=cat%20in%20the
Output: hat
'''
@app.route('/t0pp',methods = ['POST', 'GET'])
def t0pp():
    prompt = request.args.get('prompt')
    os.environ['AWS_PROFILE'] = "default"

    print('🔔 API Call!', flush=True)

    HUGGINGFACE_API_TOKEN = os.environ.get('HUGGINGFACE_API_TOKEN')
    API_URL = "https://api-inference.huggingface.co/models/bigscience/T0pp"
    headers = {"Authorization": "Bearer " + HUGGINGFACE_API_TOKEN}

    def query(payload):
        response = requests.post(API_URL, headers=headers, json=payload)
        return response.json()
        
    output = query({
        "inputs": prompt,
    })

    return jsonify(output)

# return status 200
@app.route('/')
def hello_world():
    result = {'status': 'success'}
    return result, 200


if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=8080)
