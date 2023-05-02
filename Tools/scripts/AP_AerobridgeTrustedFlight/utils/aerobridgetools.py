#!/usr/bin/env python3
'''
helper class to communicate with the Aerobridge Management server to download PEM files
'''

import requests
from uuid import UUID
# Version: 0.0.1

class AuthorityCredentialsGetter():
	"""
	All calls to your Aerobridge instance requires credentials from a oauth server, in this case this is Flight Passport OAUTH server, this class gets the token and the associated public key.
	...

	Attributes
	----------
	client_id : str
		OAUTH client id for the client credentials token
	client_secret : str
		OAUTH client secret for the client credentials token client as is set in the 
	audience : str
		The audience as set in the OAUTH server represented as a string
	base_url : str
		The fully qualified base url of the OAUTH server
	token_url : str
		The REST endpoint of the getting a token by executing a client credentials grant
	jwks_url : str
		The endpoint where the OAUTH server provides a the public key as represented in JWKS

	Methods
	-------
	get_credentials():
		Exceutes a client credentials grant request to the auth server to get a JWT token
	get_public_key():
		Gets the public key in the form of a JWKS JSON object of the OAUTH server
	"""

	def __init__(self, client_id:str, client_secret:str, audience:str, base_url:str, token_endpoint:str, jwks_endpoint:str):

		"""
        Constructs all the necessary attributes for the aerobridge object.

        Parameters
        ----------
		client_id : str
			A valid JWT Token as base64url encoded string
		audience : str
			The audience as set in the OAUTH server represented as a string
		base_url : str
			The fully qualified base url of the OAUTH server
		token_endpoint : str
			The REST endpoint of the getting a token by executing a client credentials grant
		jwks_endpoint : str
			The endpoint where the OAUTH server provides a the public key as represented in JWKS

        """
		self.client_id = client_id
		self.client_secret = client_secret
		self.audience = audience
		self.base_url = base_url
		self.token_url = base_url + token_endpoint		
		self.jwks_url = base_url + jwks_endpoint

	def get_credentials(self):  
		"""
		Excutes the Client Credentials Grant

		If the argument 'additional' is passed, then it is appended after the main info.

		Returns
		-------
		JSON with access token if the request is successful, if it is unsuccessful the JSON returned a error message is displayed 
		"""
		payload = {"grant_type":"client_credentials","client_id": self.client_id,"client_secret": self.client_secret,"audience": self.audience,"scope": 'aerobridge.read aerobridge.write'}          
		url = self.token_url
		
		token_data = requests.post(url, data = payload)
		t_data = token_data.json()     
		return t_data
		
	def get_public_key(self):          
		"""
        Gets the public key as expressed in JWKS format 

        Returns
        -------
        JSON in the JWKS format
		"""
		url = self.jwks_url		
		jwks_data = requests.get(url)
		jwks_data = jwks_data.json()     
		return jwks_data

class AerobridgeClient():
	"""
	This a a Python client that make calls to the Aerobridge API and returns data. It requires the requests package and the json module. 

	...

	Attributes
	----------
		token : str
			A valid JWT Token
		aerobridge_url : str
			The fully qualified domain name for your Aerobridge Instance
		authority_url : str
			The base url 

	"""


	def __init__(self, aerobridge_url:str, token:str= None):
		"""
        Constructs all the necessary attributes for the aerobridge object.

        Parameters
        ----------
		token : str
			A valid JWT Token as base64url encoded string
		aerobridge_url : str
			The fully qualified domain name for your Aerobridge Instance
        """
		self.token = token
		self.aerobridge_url = aerobridge_url if aerobridge_url else 'https://aerobridgetestflight.herokuapp.com/'		
		self.session = requests.Session()		
		
	def ping_aerobridge(self):
		'''
		This method checks the server heartbeat

            Returns:
                    hearbeat_response (str): Pong if the server is running properly https://redocly.github.io/redoc/?url=https://raw.githubusercontent.com/openskies-sh/aerobridge/master/api/aerobridge-1.0.0.resolved.yaml#/paths/~1ping/get
		'''

		aerobridge_url = self.aerobridge_url+ 'ping/'
		if self.token:
			headers = {'Authorization': 'Bearer '+ self.token}
		else:
			headers = {}
		r = self.session.get(aerobridge_url, headers=headers)
		return r
        
	def get_auth_server_fullchain_url(self):
		'''
		This method 

            Returns:
                     full_chain_url_details (json): A link with the full chain PEM file of the domain of the auth server that issues the permission tokens for more information see: https://redocly.github.io/redoc/?url=https://raw.githubusercontent.com/openskies-sh/aerobridge/master/api/aerobridge-1.0.0.resolved.yaml#tag/Credentials/operation/listAuthServerFullChain
		'''

		auth_server_fullchain_url = self.aerobridge_url+ 'pki/auth_server_fullchain/'
		if self.token:
			headers = {'Authorization': 'Bearer '+ self.token}
		else:
			headers = {}
		r = self.session.get(auth_server_fullchain_url, headers=headers)
		return r
        
	def download_flight_permission(self, operation_id:UUID):
		'''
		This method downloads the flight plan given a flight plan_id 

            Parameters:
                    operation_id (uuid): The uuid of a flight operation in your Aerobridge instance

            Returns:
                    plan_details (json): Details of a flight plan, see https://redocly.github.io/redoc/?url=https://raw.githubusercontent.com/openskies-sh/aerobridge/master/api/aerobridge-1.0.0.resolved.yaml#operation/retrieveFlightPlan
		'''

		securl = self.aerobridge_url + 'gcs/flight-operations/' + operation_id + '/permission'
		if self.token:
			headers = {'Authorization': 'Bearer '+ self.token}
		else:
			headers = {}
		r = self.session.put(securl, headers= headers)
		return r

	def upload_flight_log(self, operation_id:UUID, raw_log:str):
		'''
		This method uploads a flight log associated with a operation ID

            Parameters:
                    operation_id (uuid): The uuid of a flight operation in your Aerobridge instance
                    raw_log (str): The raw log file as retrieved from the vehicle.

            Returns:
                    log_details (json): Details of the newly created flight log, see https://redocly.github.io/redoc/?url=https://raw.githubusercontent.com/openskies-sh/aerobridge/master/api/aerobridge-1.0.0.resolved.yaml#operation/createFlightLog
		'''

		securl = self.aerobridge_url + 'gcs/flight-logs'
		if self.token:
			headers = {'Authorization': 'Bearer '+ self.token}
		else:
			headers = {}
		payload = {'operation':operation_id, 'raw_log':raw_log}
		r = self.session.post(securl, headers= headers,json = payload)
		return r

	def download_flight_plan(self, plan_id):
		'''
		This method downloads the flight plan given a flight plan_id 

            Parameters:
                    plan_id (uuid): The uuid of a flight plan in your Aerobridge instance

            Returns:
                    plan_details (json): Details of a flight plan, see https://redocly.github.io/redoc/?url=https://raw.githubusercontent.com/openskies-sh/aerobridge/master/api/aerobridge-1.0.0.resolved.yaml#operation/retrieveFlightPlan
		'''

		securl = self.aerobridge_url + 'gcs/flight-plans/' + plan_id
		headers = {'Authorization': 'Bearer '+ self.token, 'content-type': 'application/json'}
		r = self.session.get(securl, headers= headers)
		return r

	def get_aircraft_by_flight_controller_id(self, registered_flight_module_id:str):
		'''
		This method downloads the details of an aircraft given the flight controller ID

            Parameters:
                    registered_flight_module_id (str): The id of the flight controller 

            Returns:
                    aircraft_detail (json): Details of an aircraft, see https://redocly.github.io/redoc/?url=https://raw.githubusercontent.com/openskies-sh/aerobridge/master/api/aerobridge-1.0.0.resolved.yaml#operation/Get%20Single%20Aircraft%20(RFM%20ID)
		'''
		
		securl = self.aerobridge_url + 'registry/aircraft/rfm/' + registered_flight_module_id
		if self.token:
			headers = {'Authorization': 'Bearer '+ self.token , 'content-type': 'application/json'}
		else:
			headers = {}

		
		r = self.session.get(securl, headers= headers)
		return r
		
	def get_firmware_by_flight_controller_id(self, registered_flight_module_id:str):
		'''
		This method downloads the details of an aircraft firmware given the flight controller ID

            Parameters:
                    registered_flight_module_id (str): The id of the flight controller 

            Returns:
                    firmware_detail (json): Details of an aircraft firmware, see https://redocly.github.io/redoc/?url=https://raw.githubusercontent.com/openskies-sh/aerobridge/master/api/aerobridge-1.0.0.resolved.yaml#operation/Get%20Aircraft%20firmware%20by%20RFM%20ID
		'''
		
		
		securl = self.aerobridge_url + 'registry/aircraft/firmware/' + registered_flight_module_id
		if self.token:
			headers = {'Authorization': 'Bearer '+ self.token , 'content-type': 'application/json'}
		else:
			headers = {}
		
		r = self.session.get(securl, headers= headers)
		return r
