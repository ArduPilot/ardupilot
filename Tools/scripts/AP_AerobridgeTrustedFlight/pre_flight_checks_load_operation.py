#!/usr/bin/env python3
'''
This script downloads operation details, permission JWT and the public key credentials of the auth server signing 
the permission to be sent to the vehicle.

NOTE: The trusted flights module works independently of Aerobridge. However, Aerobridge provides digital infrastructure 
'''

import asyncio
from mavsdk import System
import utils.aerobridgetools
import json
import requests
import logging
import sys
from os import environ as env
from dotenv import load_dotenv, find_dotenv
import jwt
from cryptography.hazmat.primitives import serialization
import argparse
import tempfile
import os

parser = argparse.ArgumentParser(description='Load mission into drone and arm')
parser.add_argument("-o", "--operation_id", type=str, help ="Specify a Aerobridge Flight Operation ID, e.g. try 3408bce9-dbab-4665-abfc-8ea03b0ad871")
# An environment file stores the details of the Aerobridge Management Server Access.
load_dotenv(find_dotenv()) 
ENV_FILE = find_dotenv()
if ENV_FILE:
    load_dotenv(ENV_FILE)

logging.basicConfig(
    format="%(asctime)s %(levelname)s:%(name)s: %(message)s",
    level=logging.DEBUG,
    datefmt="%H:%M:%S",
    stream=sys.stderr,
)
# Set logging level for messages
logger = logging.getLogger("mavlink-aerobridge")
logging.getLogger("chardet.charsetprober").disabled = True
logger.setLevel(logging.INFO)

def generate_public_key_pem(jwks):
    # This method converts the JWKS JSON of the auth server return a bytes in PEM format that can be sent to the vehicle
    pem = None
    public_keys = {}
    for jwk in jwks['keys']:
        kid = jwk['kid']
        public_keys[kid] = jwt.algorithms.RSAAlgorithm.from_jwk(json.dumps(jwk))
    passport_public_key_id = env.get('PASSPORT_PUBLIC_KEY_ID', None)
    try: 
        assert passport_public_key_id in public_keys
    except AssertionError as ae:
        logging.error("Public key ID %s not found in the JWKS" % passport_public_key_id)
        exit()
    else:
        public_key = public_keys[passport_public_key_id]
        pem = public_key.public_bytes(
                encoding=serialization.Encoding.PEM,
                format=serialization.PublicFormat.SubjectPublicKeyInfo)       
    return pem

async def run(operation_id):    
    # This is the main function to execute the preflight checks, for loading the operation that is stored in Aerobridge .Aerobridge issues permissions as a JWT token that is downloaded.
    my_authorization_helper = aerobridgetools.AuthorityCredentialsGetter(client_id = env.get('AEROBRIDGE_CLIENT_ID', None), client_secret= env.get('AEROBRIDGE_CLIENT_SECRET', None), audience = env.get('AEROBRIDGE_AUDIENCE', None), base_url= env.get('PASSPORT_URL') ,token_endpoint = env.get('PASSPORT_TOKEN_ENDPOINT'), jwks_endpoint=env.get('PASSPORT_JWKS_ENDPOINT'))
    logging.info("Getting token from Authority server")

    # Connect to the vehicle
    vehicle = System()
    await vehicle.connect(system_address=env.get('SYSTEM_ADDRESS', "udp://:14540"))

    async for state in vehicle.core.connection_state():
        if state.is_connected:
            print(f"Vehicle discovered")
            break
    
    ## Get the Auth Server related data: Public Key of the Auth server and Full Chain PEM details of the Auth server that is sent to the vehicle

    # Created Trusted Flight Directory on the vehicle
    logging.info("Creating 'trusted_flight' directory on the board..")
    trusted_flight_directory = await vehicle.ftp.create_directory("trusted_flight")
    logging.info("Trusted flight directory successfully created!")

    # Get token for accessing Aerobridge Management Server
    auth_token = my_authorization_helper.get_credentials()       
    try: 
        assert 'access_token' in auth_token.keys()
    except AssertionError as ae: 
        logging.info("Error in getting access token from auth server %s" % auth_token)
        print("Error in getting access token")
        exit()
    else:
        logging.info("Successfully retrieved access token") 
    logging.info("Getting public key from Authority server")
    # Get the Authorization server Public Key
    jwks = my_authorization_helper.get_public_key()    
    # Convert public key to PEM and send to drone
    logging.debug("Converting JWKS to PEM format")
    pem = generate_public_key_pem(jwks)
    logging.debug("JWKS to PEM conversion successful")    

    # Create a temporary file to hold Auth Server PEM and send it to the drone
    pem_file = tempfile.NamedTemporaryFile(prefix='auth_server_public_key', suffix='.pem',delete = False)

    try:
        logging.debug("Writing PEM file data")
        with open(pem_file.name, 'wb') as f:
            f.write(pem)
    except Exception as e: 
        logging.error("Error in writing file details %s" % e)
        exit()
    else:
        # send file to drone         
        logging.info("Uploading public key to the drone")
        pem_upload = vehicle.ftp.upload(pem_file.name, trusted_flight_directory)
        logging.info("PEM File uploaded successfully")
    finally:
        pem_file.close()
        os.unlink(pem_file.name)
        logging.debug("PEM file succesfully deleted")


    # Download the Server Full Chain PEM
    auth_server_full_chain_query = my_aerobridge_client.get_auth_server_fullchain_url()
    if auth_server_full_chain_query.status_code ==200: 
        # For more information see: https://redocly.github.io/redoc/?url=https://raw.githubusercontent.com/openskies-sh/aerobridge/master/api/aerobridge-1.0.0.resolved.yaml#tag/Credentials/operation/listAuthServerFullChain
        auth_server_full_chain_url = auth_server_full_chain_query['binary_file_url']
    else:
        print("Auth server full chain PEM could not be downloaded")
        exit()

    # Create a file to hold permission the auth server full chain
    full_chain_file = tempfile.NamedTemporaryFile(prefix='fullchain', suffix='.pem' delete = False)
    full_chain_download_query = requests.get(auth_server_full_chain_url, stream=True)

    if full_chain_download_query.status_code == 200: 
        try:
            with open(full_chain_file.name, 'wb') as f:
                for chunk in full_chain_download_query.iter_content(chunk_size=1024): 
                    if chunk: 
                        f.write(chunk)

        except Exception as e: 
            logging.error("Error in writing full chain data to file %s" %e)
            exit()
        else:
            # send file to drone 
            full_chain_upload = vehicle.ftp.upload(full_chain_file.name, trusted_flight_directory)
        finally:
            full_chain_file.close()
            os.unlink(full_chain_file.name)
    else:
        logging.error("Error in writing full chain data to file %s" %e)
        exit()

    # Create a Aerobridge Client
    my_aerobridge_client = aerobridgetools.AerobridgeClient(aerobridge_url= env.get('AEROBRIDGE_URL'), token=auth_token['access_token'])
    permission_granted = False
    
    plan_id = '0'
    
    # Download flight permission, its bascially a JWT token
    operation_permission= my_aerobridge_client.download_flight_permission(operation_id=operation_id)
    if operation_permission.status_code == 200:
        logging.info("Successfully found permission with operation id %s in management server" % operation_id)
        permission_data = operation_permission.json()
        logging.info("Permission for flight successfully retrieved") 
        
        if permission_data['status_code'] == "granted":
            logging.info("Permission has been issued for this flight")
            permission_granted = True                
            plan_id = permission_data['operation']['flight_plan']
        else:
            print("Cannot arm drone, permission status is %s" % permission_data['status_code'])                
            exit()
    mission_plan_data = False

    if permission_granted:
        # Flight permission has been granted download the mission plan 
        # All checks passed, download flight operation permission and public key    
        geo_cage = permission_data['geo_cage']
        flight_plan_details = my_aerobridge_client.download_flight_plan(plan_id=plan_id)
        if flight_plan_details.status_code == 200:
            logging.info("Successfully found flight plan with id %s in management server" % plan_id)
            flight_plan_data = flight_plan_details.json()
            logging.info("Plan data for mission successfully retrieved") 
            mission_plan_data = flight_plan_data['plan_file_json']
        else:
            print("Plan data for the operation cannot be retrieved")
            exit()

    # Create a file to hold permission OTP
    auth_token_file = tempfile.NamedTemporaryFile(prefix='aerobridge_trusted_flight.jwt', suffix='.json', delete = False)
    permission_token = permission_data['token']
    try:
        with open(auth_token_file.name, 'w') as f:
            f.writelines(json.dumps(permission_token))
    except Exception as e: 
        logging.error("Error in writing auth token to file %s" %e)
        exit()
    else:
        # send file to drone 
        auth_token_upload = vehicle.ftp.upload(auth_token_file.name, trusted_flight_directory)
    finally:
        auth_token_file.close()
        os.unlink(auth_token_file.name)
    
    mission_plan_data = False
    # Send mission data
    if mission_plan_data: 
        mission_file = tempfile.NamedTemporaryFile(delete=False)
        try:
            with open(mission_file.name, 'w') as f:
                f.writelines(json.dumps(mission_plan_data))
        except Exception as e: 
            logging.error("Error in writing file details")
            exit()
        else:
            mission_import_data = await vehicle.mission_raw.import_qgroundcontrol_mission(mission_file.name)
            print(f"{len(mission_import_data.mission_items)} mission items imported")
        finally:
            mission_file.close()
            os.unlink(mission_file.name)
        
        logging.info("Uploading mission file to the aircraft..")
        await vehicle.mission_raw.upload_mission(mission_import_data.mission_items)
        await vehicle.mission.set_return_to_launch_after_mission(True)
        logging.info("Mission File uploaded")
        print("Mission uploaded succesfully")
   

    # All checks done, drone ready to be armed. 
    print("All checks passed, flight permission, public key, full certificate chain and mission transferred to the drone.")
    # Try to arm the drone
    print("-- Arming")
    await vehicle.action.arm()
    
if __name__ == '__main__':
    # Start the main function
    args = parser.parse_args()
    operation_id = args.operation_id
    if not operation_id:
        print("A valid Operation ID (UUID) from your Aerobridge instance must be provided before arming the drone with a mission, e.g. try -o 3408bce9-dbab-4665-abfc-8ea03b0ad871" )
        exit()

    loop = asyncio.get_event_loop()
    asyncio.set_event_loop(loop)
    tasks = asyncio.gather(run(operation_id))
    try:
        loop.run_until_complete(tasks)
    except (Exception, KeyboardInterrupt) as e:
        print('ERROR', str(e))
        exit()
