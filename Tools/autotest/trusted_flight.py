'''
Validate Trusted Flight in SITL

AP_FLAKE8_CLEAN
'''

import atexit
import functools
import os
import shutil

from enum import Enum

from arducopter import AutoTestCopter


class TokenNameMappings(str, Enum):
    valid_token = 'valid'
    invalid_token = 'invalid'
    token_with_invalid_base64 = 'invalid_base64'
    token_with_invalid_json = 'invalid_json'
    token_without_payload = 'no_payload'
    token_without_signature = 'no_signature'
    token_without_typ = 'missing_typ'
    token_with_invalid_typ = 'invalid_typ'
    token_without_alg = 'missing_alg'
    token_with_invalid_alg = 'invalid_alg'
    token_without_iss = 'missing_iss'
    token_with_invalid_iss = 'invalid_iss'
    token_without_iat = 'missing_iat'
    token_with_invalid_iat = 'invalid_iat'
    token_with_future_iat = 'iat_in_future'
    token_without_nbf = 'missing_nbf'
    token_with_invalid_nbf = 'invalid_nbf'
    token_with_future_nbf = 'nbf_in_future'
    token_without_exp = 'missing_exp'
    token_with_invalid_exp = 'invalid_exp'
    token_with_past_exp = 'exp_in_past'


class TrustedFlightTestsWrapper:

    @staticmethod
    def test_artifacts_directory():
        return 'build/trusted_flight_artifacts/autotest/'

    @staticmethod
    def token_storage_path():
        os.makedirs('trusted_flight', exist_ok=True)
        return 'trusted_flight/'

    def __init__(self, token_file=None):
        self._token_file = token_file

    def __call__(self, func):
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            try:
                if self._token_file is not None:
                    # copy token for trusted flight test
                    shutil.copy(f'{self.test_artifacts_directory()}/{self._token_file}',
                                f'{self.token_storage_path()}/token')
                func(*args, **kwargs)
            finally:
                # cleanup any artifacts after the test run
                shutil.rmtree(self.token_storage_path())
        return wrapper


class AutoTestTrustedFlight(AutoTestCopter):

    def __init__(self, *args, **kwargs):
        def _prepare_test_artifacts(artifacts_node):
            import sys

            sys.path.append(os.path.join(os.path.dirname(__file__), '../../', 'libraries/AP_TrustedFlight/tools'))
            from generate_key_and_token import create_key_pair, create_token, create_invalid_token, \
                TOKEN_ISSUER, JWT_TYPE, JWT_ALG

            import contextlib
            from datetime import datetime, timezone, timedelta

            with open(os.devnull, 'w') as null, contextlib.redirect_stdout(null), contextlib.redirect_stderr(null):
                pk_pub_path = f'{artifacts_node}/valid.key'
                pk = create_key_pair(pk_pub_path)
                create_token(pk, f'{artifacts_node}/valid')
                create_invalid_token(pk, f'{artifacts_node}/invalid_base64', encode=False)
                create_invalid_token(pk, f'{artifacts_node}/invalid_json')
                create_token(pk, f'{artifacts_node}/no_payload', include_payload=False)
                create_token(pk, f'{artifacts_node}/no_signature', include_signature=False)
                create_token(pk, f'{artifacts_node}/missing_typ', header={
                    'alg': JWT_ALG,
                })
                create_token(pk, f'{artifacts_node}/invalid_typ', header={
                    'typ': 'invalid_typ',
                    'alg': JWT_ALG,
                })
                create_token(pk, f'{artifacts_node}/missing_alg', header={
                    'typ': JWT_TYPE,
                })
                create_token(pk, f'{artifacts_node}/invalid_alg', header={
                    'typ': JWT_TYPE,
                    'alg': "RSA",
                })
                create_token(pk, f'{artifacts_node}/missing_iss', payload={
                    'iat': datetime.now(timezone.utc),
                    'exp': datetime.now(timezone.utc) + timedelta(minutes=5),
                })
                create_token(pk, f'{artifacts_node}/invalid_iss', payload={
                    'iss': 'test_issuer',
                    'iat': datetime.now(timezone.utc),
                    'exp': datetime.now(timezone.utc) + timedelta(minutes=5),
                })
                create_token(pk, f'{artifacts_node}/missing_iat', payload={
                    'iss': TOKEN_ISSUER,
                    'exp': datetime.now(timezone.utc) + timedelta(minutes=5),
                })
                create_token(pk, f'{artifacts_node}/invalid_iat', payload={
                    'iss': TOKEN_ISSUER,
                    'iat': str(datetime.now(timezone.utc)),
                    'exp': datetime.now(timezone.utc) + timedelta(minutes=5),
                })
                create_token(pk, f'{artifacts_node}/iat_in_future', payload={
                    'iss': TOKEN_ISSUER,
                    'iat': datetime.now(timezone.utc) + timedelta(hours=1),
                    'exp': datetime.now(timezone.utc) + timedelta(minutes=5),
                })
                create_token(pk, f'{artifacts_node}/missing_nbf', payload={
                    'iss': TOKEN_ISSUER,
                    'iat': datetime.now(timezone.utc),
                    'exp': datetime.now(timezone.utc) + timedelta(minutes=5),
                })
                create_token(pk, f'{artifacts_node}/invalid_nbf', payload={
                    'iss': TOKEN_ISSUER,
                    'iat': datetime.now(timezone.utc),
                    'nbf': str(datetime.now(timezone.utc)),
                    'exp': datetime.now(timezone.utc) + timedelta(minutes=5),
                })
                create_token(pk, f'{artifacts_node}/nbf_in_future', payload={
                    'iss': TOKEN_ISSUER,
                    'iat': datetime.now(timezone.utc),
                    'nbf': datetime.now(timezone.utc) + timedelta(minutes=5),
                    'exp': datetime.now(timezone.utc) + timedelta(minutes=6),
                })
                create_token(pk, f'{artifacts_node}/missing_exp', payload={
                    'iss': TOKEN_ISSUER,
                    'iat': datetime.now(timezone.utc),
                })
                create_token(pk, f'{artifacts_node}/invalid_exp', payload={
                    'iss': TOKEN_ISSUER,
                    'iat': datetime.now(timezone.utc),
                    'exp': str(datetime.now(timezone.utc) + timedelta(minutes=5)),
                })
                create_token(pk, f'{artifacts_node}/exp_in_past', payload={
                    'iss': TOKEN_ISSUER,
                    'iat': datetime.now(timezone.utc),
                    'exp': datetime.now(timezone.utc) - timedelta(minutes=2),
                })

                another_pk_pub_path = f'{artifacts_node}/another.key'

                pk = create_key_pair(another_pk_pub_path)
                create_token(pk, f'{artifacts_node}/invalid')

        os.makedirs(f'{TrustedFlightTestsWrapper.test_artifacts_directory()}', exist_ok=True)
        _prepare_test_artifacts(f'{TrustedFlightTestsWrapper.test_artifacts_directory()}')

        os.makedirs(TrustedFlightTestsWrapper.token_storage_path(), exist_ok=True)
        shutil.copy(f'{TrustedFlightTestsWrapper.test_artifacts_directory()}/valid.key',
                    f'{TrustedFlightTestsWrapper.token_storage_path()}/key.pub')
        with open(f'{TrustedFlightTestsWrapper.token_storage_path()}/token_issuer', 'w') as f:
            f.write('test.cname')

        super(AutoTestTrustedFlight, self).__init__(*args, **kwargs)

        atexit.register(self.cleanup)

    def cleanup(self):
        # cleanup test tokens and keys from current run
        shutil.rmtree(TrustedFlightTestsWrapper.test_artifacts_directory())

    def force_arm_fail(self, fail_msg):
        self.context_push()
        try:
            self.wait_ready_to_arm(check_prearm_bit=False)
            self.assert_arm_failure(f'Arm: TrustedFlight: {fail_msg}', force=True)
        finally:
            self.disarm_vehicle(force=True)
            self.context_pop()

    def prearm_fail(self, fail_msg, force=False):
        self.context_push()
        self.assert_prearm_failure(f'PreArm: TrustedFlight: {fail_msg}', other_prearm_failures_fatal=False)
        self.context_pop()

    def arming_successful(self):
        self.context_push()
        try:
            self.wait_ready_to_arm()
            self.arm_vehicle()
        except Exception as e:
            self.print_exception_caught(e)
            raise
        finally:
            self.disarm_vehicle(force=True)
            self.context_pop()

    @TrustedFlightTestsWrapper()
    def MissingToken(self):
        '''Token is not provided'''
        self.prearm_fail('Unable to read token')

    @TrustedFlightTestsWrapper(token_file=TokenNameMappings.token_with_invalid_base64)
    def TokenWithInvalidBase64(self):
        '''Token with invalid base64 encoding'''
        self.prearm_fail('Invalid token format')

    @TrustedFlightTestsWrapper(token_file=TokenNameMappings.token_with_invalid_json)
    def TokenWithInvalidJson(self):
        '''Token with invalid json'''
        self.prearm_fail('Invalid token format')

    @TrustedFlightTestsWrapper(token_file=TokenNameMappings.token_without_payload)
    def TokenWithoutPayload(self):
        '''Token without payload'''
        self.prearm_fail('Invalid token format')

    @TrustedFlightTestsWrapper(token_file=TokenNameMappings.token_without_signature)
    def TokenWithoutSignature(self):
        '''Token without signature'''
        self.prearm_fail('Invalid token format')

    @TrustedFlightTestsWrapper(token_file=TokenNameMappings.token_without_typ)
    def TokenWithoutType(self):
        '''Token without typ claim in header'''
        self.prearm_fail('Invalid token type')

    @TrustedFlightTestsWrapper(token_file=TokenNameMappings.token_with_invalid_typ)
    def TokenWithInvalidType(self):
        '''Token with invalid typ claim in header'''
        self.prearm_fail('Invalid token type')

    @TrustedFlightTestsWrapper(token_file=TokenNameMappings.token_without_alg)
    def TokenWithoutAlgorithm(self):
        '''Token without alg claim in header'''
        self.prearm_fail('Invalid token algorithm')

    @TrustedFlightTestsWrapper(token_file=TokenNameMappings.token_with_invalid_alg)
    def TokenWithInvalidAlgorithm(self):
        '''Token with invalid alg claim in header'''
        self.prearm_fail('Invalid token algorithm')

    @TrustedFlightTestsWrapper(token_file=TokenNameMappings.token_without_iss)
    def TokenWithoutIssuer(self):
        '''Token without iss claim in payload'''
        self.prearm_fail('Invalid token issuer')

    @TrustedFlightTestsWrapper(token_file=TokenNameMappings.token_with_invalid_iss)
    def TokenWithInvalidIssuer(self):
        '''Token with invalid iss claim in payload'''
        self.prearm_fail('Invalid token issuer')

    @TrustedFlightTestsWrapper(token_file=TokenNameMappings.token_without_iat)
    def TokenWithoutIat(self):
        '''Token without iat claim in payload'''
        self.prearm_fail('Invalid token iat claim')

    @TrustedFlightTestsWrapper(token_file=TokenNameMappings.token_with_invalid_iat)
    def TokenWithInvalidIat(self):
        '''Token with invalid iat claim in payload'''
        self.prearm_fail('Invalid token iat claim')

    @TrustedFlightTestsWrapper(token_file=TokenNameMappings.token_with_future_iat)
    def TokenWithIatInFuture(self):
        '''Token with issuance time in future'''
        self.prearm_fail('Invalid token iat claim')

    @TrustedFlightTestsWrapper(token_file=TokenNameMappings.token_without_nbf)
    def TokenWithoutNbf(self):
        '''Token without nbf claim in payload'''
        self.arming_successful()

    @TrustedFlightTestsWrapper(token_file=TokenNameMappings.token_with_invalid_nbf)
    def TokenWithInvalidNbf(self):
        '''Token with invalid nbf claim in payload'''
        self.prearm_fail('Invalid token nbf claim')

    @TrustedFlightTestsWrapper(token_file=TokenNameMappings.token_with_future_nbf)
    def TokenWithNbfInFuture(self):
        '''Token with notBefore in future'''
        self.prearm_fail('Invalid token nbf claim')

    @TrustedFlightTestsWrapper(token_file=TokenNameMappings.token_without_exp)
    def TokenWithoutExp(self):
        '''Token without exp claim in payload'''
        self.prearm_fail('Invalid token exp claim')

    @TrustedFlightTestsWrapper(token_file=TokenNameMappings.token_with_invalid_exp)
    def TokenWithInvalidExp(self):
        '''Token with invalid exp claim in payload'''
        self.prearm_fail('Invalid token exp claim')

    @TrustedFlightTestsWrapper(token_file=TokenNameMappings.token_with_past_exp)
    def TokenWithExpInPast(self):
        '''Token with expiration in past'''
        self.prearm_fail('Invalid token exp claim')

    @TrustedFlightTestsWrapper(token_file=TokenNameMappings.invalid_token)
    def TokenSignedWithDifferentKey(self):
        '''Token signed with another keypair'''
        self.prearm_fail('Invalid token signature')

    @TrustedFlightTestsWrapper(token_file=TokenNameMappings.invalid_token)
    def TokenSignedWithDifferentKeyForceArm(self):
        '''Force arm with token signed with another keypair'''
        self.force_arm_fail('Invalid token signature')

    @TrustedFlightTestsWrapper(token_file=TokenNameMappings.valid_token)
    def ValidToken(self):
        '''Vaild token'''
        self.arming_successful()

    def tests(self):
        return [
            self.MissingToken,
            self.TokenWithInvalidBase64,
            self.TokenWithInvalidJson,
            self.TokenWithoutPayload,
            self.TokenWithoutSignature,
            self.TokenWithoutType,
            self.TokenWithInvalidType,
            self.TokenWithoutAlgorithm,
            self.TokenWithInvalidAlgorithm,
            self.TokenWithoutIssuer,
            self.TokenWithInvalidIssuer,
            self.TokenWithoutIat,
            self.TokenWithInvalidIat,
            self.TokenWithIatInFuture,
            self.TokenWithoutNbf,
            self.TokenWithInvalidNbf,
            self.TokenWithNbfInFuture,
            self.TokenWithoutExp,
            self.TokenWithInvalidExp,
            self.TokenWithExpInPast,
            self.TokenSignedWithDifferentKey,
            self.TokenSignedWithDifferentKeyForceArm,
            self.ValidToken
        ]
