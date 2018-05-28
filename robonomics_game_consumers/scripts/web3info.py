# -*- coding: utf-8

from web3 import Web3
from flask import Flask, request
from flask_restful import Resource, Api
from flask.json import jsonify

app = Flask(__name__)
api = Api(app)

with open('./infura_api.key', 'r') as f:
    apikey = f.readline()
w3 = Web3(Web3.HTTPProvider('https://ropsten.infura.io/' + apikey))

class BlockNumber(Resource):
    def get(self):
        return jsonify({'blockNumber': w3.eth.blockNumber})

api.add_resource(BlockNumber, '/blockNumber')


if __name__ == '__main__':
     app.run(port='5002')
