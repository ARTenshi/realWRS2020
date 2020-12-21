import rospy
import json
import urllib2
from datetime import datetime
import time

SERVER_URL = 'http://192.168.100.2:8000/data-en.json'
TIME_FORMAT = '%Y-%m-%dT%H:%M:%S.%f'

class RospeexClient:
    def __init__(self):
        req = urllib2.Request(SERVER_URL)
        response = urllib2.urlopen(req).read()
        contents = json.loads(response)
        self.last_sr = datetime.strptime(contents[0]['time'], TIME_FORMAT)

    def next_speech(self, timeout=None):
        elapsed = 0.
        while not rospy.is_shutdown() and (not timeout or elapsed < timeout):
            req = urllib2.Request(SERVER_URL)
            response = urllib2.urlopen(req).read()
            contents = json.loads(response)
            t = datetime.strptime(contents[0]['time'], TIME_FORMAT)
            if t > self.last_sr:
                last_sr = t
                return contents[0]['transcription']
            time.sleep(.5)
            elapsed += .5
        raise Exception('Timeout while waiting for the speech recognition.')
