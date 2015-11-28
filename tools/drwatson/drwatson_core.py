#
# Copyright (C) 2015 Zubax Robotics, <info@zubax.com>
# Author: Pavel Kirienko <pavel.kirienko@zubax.com>
#

import requests
import getpass
import json
import os
import base64
import logging
import http.client as http_codes
import colorama
import sys
from functools import partial
try:
    import readline  # @UnusedImport
except ImportError:
    pass


LICENSING_ENDPOINT = os.environ.get('ZUBAX_LICENSING_ENDPOINT', 'licensing.zubax.com')
APP_DATA_PATH = os.path.join(os.path.expanduser("~"), '.zubax', 'drwatson')
REQUEST_TIMEOUT = 20


colorama.init()

logger = logging.getLogger(__name__)


class DrwatsonException(Exception):
    pass


class APIException(DrwatsonException):
    pass


class ResponseParams(dict):
    def __init__(self, *args, **kwargs):
        super(ResponseParams, self).__init__(*args, **kwargs)
        self.__dict__ = self

    def _b64_decode_existing_params(self, param_names):
        for p in param_names:
            if p in self:
                self[p] = _b64_decode(self[p])


class APIContext:
    def __init__(self, login, password):
        self.login = login
        self.password = password

    def _call(self, call, **arguments):
        logger.debug('Calling %r with %r', call, arguments)

        endpoint = _make_api_endpoint(self.login, self.password, call)
        if len(arguments):
            data = json.dumps(arguments)
            resp = requests.post(endpoint, data=data, timeout=REQUEST_TIMEOUT)
        else:
            resp = requests.get(endpoint, timeout=REQUEST_TIMEOUT)

        if resp.status_code == http_codes.PAYMENT_REQUIRED:
            raise APIException('PAYMENT REQUIRED [%s]' % resp.text)

        if resp.status_code == http_codes.BAD_REQUEST:
            raise APIException('BAD REQUEST [%s]' % resp.text)

        if resp.status_code != http_codes.OK:
            raise APIException('Unexpected HTTP code: %r [%s]' % (resp, resp.text))

        resp = resp.text
        return resp if not resp else ResponseParams(json.loads(resp))

    def get_balance(self):
        return self._call('balance')

    def generate_signature(self, unique_id, product_name):
        resp = self._call('signature/generate', unique_id=_b64_encode(unique_id), product_name=product_name)
        resp._b64_decode_existing_params(['unique_id', 'signature'])
        return resp

    def verify_signature(self, unique_id, product_name, signature):
        return self._call('signature/verify', unique_id=_b64_encode(unique_id),
                         product_name=product_name, signature=_b64_encode(signature))


def make_api_context_with_user_provided_credentials():
    # Reading login from cache
    login_cache_path = os.path.join(APP_DATA_PATH, 'licensing_login')
    try:
        with open(login_cache_path) as f:
            login = f.read().strip()
    except Exception:
        logger.debug('Could not read login cache', exc_info=True)
        login = None

    # Running in the loop until the user provides valid credentials
    while True:
        print_imperative('Enter your credentials for %r', LICENSING_ENDPOINT)

        provided_login = input(('Login [%s]: ' % login) if login else 'Login: ')
        login = provided_login or login

        password = getpass.getpass('Password: ')

        response = requests.get(_make_api_endpoint(login, password, 'balance'), timeout=REQUEST_TIMEOUT)
        if response.status_code == http_codes.UNAUTHORIZED:
            print('Incorrect credentials')
        elif response.status_code == http_codes.OK:
            break
        else:
            raise APIException('Unexpected HTTP code: %r' % response)

    print_info('Credentials are correct' if _ordinary() else 'You are a good man, we like you')

    # Trying to cache the login
    try:
        try:
            os.makedirs(APP_DATA_PATH, exist_ok=True)
        except Exception:
            logger.debug('Could not create login cache dir', exc_info=True)
        with open(login_cache_path, 'w') as f:
            f.write(login)
    except Exception:
        logger.info('Could not write login cache', exc_info=True)

    # Returning new instance with newly supplied login credentials
    return APIContext(login, password)


def download(url):
    r = requests.get(url, stream=True)
    if r.status_code == 200:
        r.raw.decode_content = True
        data = r.raw.read()
        logging.info('Downloaded %d bytes from %r', len(data), url)
        return data
    raise DrwatsonException('Could not download %r' % url)


def _print_impl(color, fmt, *args):
    sys.stdout.write(colorama.Style.BRIGHT)
    sys.stdout.write(color)
    sys.stdout.write(fmt % args)
    sys.stdout.write('\n')
    sys.stdout.write(colorama.Style.RESET_ALL)
    sys.stdout.flush()

print_imperative = partial(_print_impl, colorama.Fore.GREEN)
print_error = partial(_print_impl, colorama.Fore.RED)
print_info = partial(_print_impl, colorama.Fore.WHITE)

def show_color_legend():
    print('Color legend:')
    print_imperative('\tIMPERATIVE')
    print_error('\tERROR')
    print_info('\tINFO')


def request_input(fmt, *args):
    print_imperative(fmt, *args)
    return input()


def fatal(fmt, *args):
    print_error(fmt, *args)
    exit(1)


def run(handler):
    while True:
        try:
            print('=' * 80)
            request_input('Press ENTER to begin')

            handler()
        except Exception as ex:
            print_error('FAILED: %s: %s', type(ex).__name__, ex)
            logger.info('Main loop exception', exc_info=True)

def _make_api_endpoint(login, password, call):
    return 'https://%s:%s@%s/api/v1/%s' % (login, password, LICENSING_ENDPOINT, call)


def _b64_encode(x):
    if isinstance(x, str):
        x = x.encode('utf8')
    if not isinstance(x, bytes):
        x = bytes(x)
    return base64.b64encode(x).decode()


def _b64_decode(x):
    return base64.b64decode(x, validate=True)


def _ordinary():
    import random
    return random.random() >= 0.01


