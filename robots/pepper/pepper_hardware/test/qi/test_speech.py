#!/usr/bin/env python

import qi


def callback(recognition):
    print recognition

session = qi.Session()
session.connect("tcp://localhost:9559")

speech = session.get_service("ALSpeechRecognition")
speech.unsubscribe("test")
speech.subscribe("test")

mem = session.get_service("ALMemory")

subscriber = mem.subscriber("WordRecognized")
id = subscriber.signal.connect(callback)

speech.setAudioExpression(False)
speech.setVisualExpression(False)

# Make sure the speech recognition service is paused and clean
speech.pause(True)
speech.deleteAllContexts()
speech.setLanguage("English")

speech.compile(remote_bnf_path, remote_lcf_path, language)
speech.addContext(remote_lcf_path, "grammar")
