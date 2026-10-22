#!/usr/bin/env python3
"""
Voice assistant for the SCARA (like "Building a ROS2 SCARA Robot - Part 9").

    1. Voice input      microphone (SpeechRecognition + PyAudio)
    2. Speech to text   Whisper - offline (pip install openai-whisper) or the
                        OpenAI API (OPENAI_API_KEY), falls back to Google's free recogniser
    3. Prompt + text    the sentence is turned into a command, either by ChatGPT
                        (OpenAI API, --llm) or by a built-in keyword parser (no internet needed)
    4. Robot            the command goes to the sorter (Gazebo or MuJoCo)

Conversation:
    you:   "Hey SCARA"
    SCARA: "Yes, what would you like me to pick up?"
    you:   "red"  /  "red and blue"  /  "any"  /  "three red ones"  /  "none"  /  "stop"
    SCARA: "Okay, picking red until you tell me another colour."

This file has no ROS or MuJoCo inside: scara_ask.py (ROS 2) and
launch_mujoco.py (--command / --voice) both use it.
"""
import json
import os
import re
import shutil
import subprocess
import threading
import time

try:
    import scara_scene as S
except ImportError:
    from . import scara_scene as S

WAKE_WORDS = ('hey scara', 'hi scara', 'ok scara', 'okay scara', 'hello scara', 'scara',
              'hey sakara', 'hey scarra', 'hey sarah', 'hey scar')
SYNONYMS = {'read': 'red', 'rad': 'red', 'bread': 'red', 'grin': 'green', 'greens': 'green', 'blew': 'blue',
            'bloo': 'blue', 'yello': 'yellow', 'yellows': 'yellow', 'everything': 'all', 'every': 'all',
            'any': 'all', 'anything': 'all', 'anyone': 'all', 'next': 'all',
            'pause': 'stop', 'halt': 'stop', 'wait': 'stop', 'continue': 'start', 'resume': 'start', 'go': 'start'}
LLM_PROMPT = (
    'You control a SCARA robot next to a conveyor that carries coloured pucks (red, green, blue, yellow). '
    'When asked for a colour it keeps picking every puck of that colour and drops it in its bin, '
    'until it is given another colour ("any" / "all" = every colour, "none" = pick nothing). '
    'A number limits how many ("three red" -> colors ["3", "red"]). Turn the user\'s sentence into JSON: {"action": "pick" | "stop" | "start" | "empty" | "idle" | "none", '
    '("empty" = the user emptied the bins), '
    '"colors": [list of colours, or ["all"]], "reply": "one short friendly sentence to say back"}. '
    'Only output the JSON.')


NUMBERS = {'two': 2,   # not 'one': "the red one" means red
           'three': 3, 'four': 4, 'five': 5, 'six': 6,
           'seven': 7, 'eight': 8, 'nine': 9, 'ten': 10}


def parse_command(text):
    """
    Keyword parser -> 'stop', 'start', 'none', 'all', 'red blue', '3 red',
    'empty [colours]' ... or None.
    A colour stays active until the next one ("red" = every red puck);
    a number limits it ("three red ones" -> '3 red').
    """
    raw = re.findall(r'[a-z]+|[0-9]+', text.lower())
    words = [SYNONYMS.get(w, w) for w in raw]
    if 'stop' in words:
        return 'stop'
    if any(w in ('empty', 'emptied', 'clear', 'cleared') for w in words):
        cols = [w for w in words if w in S.COLORS]
        return ' '.join(['empty'] + list(dict.fromkeys(cols)))
    if any(w in ('none', 'nothing', 'cancel', 'idle') for w in words):
        return 'none'
    colors = [w for w in words if w in S.COLORS]
    if colors and ('then' in words or any(w in ('sequence', 'order') for w in words)):
        seq = []                                   # "red then blue then two green" -> sequence red blue 2 green
        for w in raw:
            w2 = SYNONYMS.get(w, w)
            if w.isdigit() or w in NUMBERS:
                seq.append(str(int(w) if w.isdigit() else NUMBERS[w]))
            elif w2 in S.COLORS:
                seq.append(w2)
        return 'sequence ' + ' '.join(seq)
    num = None
    if colors or 'all' in words:
        for w in raw:
            if w.isdigit() or w in NUMBERS:
                num = int(w) if w.isdigit() else NUMBERS[w]
                break
    prefix = f'{num} ' if num else ''
    if 'all' in words:
        return prefix + 'all'
    if colors:
        return prefix + ' '.join(dict.fromkeys(colors))
    if 'start' in words:
        return 'start'
    return None


def strip_wake_word(text):
    t = ' ' + re.sub(r'[^a-z ]', ' ', text.lower()) + ' '
    for w in sorted(WAKE_WORDS, key=len, reverse=True):
        if f' {w} ' in t:
            return True, t.replace(f' {w} ', ' ', 1).strip()
    return False, text.lower().strip()


class Speaker:
    """Text to speech: pyttsx3, else espeak / spd-say, else just print."""

    def __init__(self, enabled=True):
        self.engine = None
        self.cmd = None
        if not enabled:
            return
        try:
            import pyttsx3
            self.engine = pyttsx3.init()
            self.engine.setProperty('rate', 170)
        except Exception:
            for prog in ('espeak-ng', 'espeak', 'spd-say'):
                if shutil.which(prog):
                    self.cmd = prog
                    break

    def say(self, text):
        print(f'SCARA: {text}', flush=True)
        try:
            if self.engine:
                self.engine.say(text)
                self.engine.runAndWait()
            elif self.cmd:
                subprocess.run([self.cmd, text], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=15)
        except Exception:
            pass


class LLMParser:
    """Optional ChatGPT understanding (needs `pip install openai` and OPENAI_API_KEY)."""

    def __init__(self, model='gpt-4o-mini'):
        from openai import OpenAI          # raises if not installed
        if not os.environ.get('OPENAI_API_KEY'):
            raise RuntimeError('OPENAI_API_KEY is not set')
        self.client = OpenAI()
        self.model = model

    def parse(self, text):
        r = self.client.chat.completions.create(
            model=self.model, temperature=0,
            messages=[{'role': 'system', 'content': LLM_PROMPT}, {'role': 'user', 'content': text}])
        data = json.loads(r.choices[0].message.content.strip().strip('`').replace('json\n', '', 1))
        action = data.get('action', 'none')
        if action == 'stop':
            return 'stop', data.get('reply')
        if action == 'start':
            return 'start', data.get('reply')
        if action == 'empty':
            cols = [c for c in data.get('colors', []) if c in S.COLORS]
            return ' '.join(['empty'] + cols), data.get('reply')
        if action == 'idle':
            return 'none', data.get('reply')
        raw = [str(c).lower() for c in data.get('colors', [])]
        cols = [c for c in raw if c in S.COLORS or c in ('all', 'any')]
        num = next((c for c in raw if c.isdigit()), '')
        if action == 'pick' and cols:
            what = 'all' if ('all' in cols or 'any' in cols) else ' '.join(cols)
            return (f'{num} {what}' if num else what), data.get('reply')
        return None, data.get('reply')


class VoiceAssistant:
    """
    on_command(cmd) is called with 'red', 'red blue', 'all', 'stop' or 'start'.
    feed(text) drives the conversation (from the microphone or from typing).
    """

    def __init__(self, on_command, speak=True, require_wake=True, use_llm=False, listen_timeout=10.0):
        self.on_command = on_command
        self.speaker = Speaker(speak)
        self.require_wake = require_wake
        self.listen_timeout = listen_timeout
        self.awake_until = 0.0
        self.llm = None
        if use_llm:
            try:
                self.llm = LLMParser()
            except Exception as e:
                print(f'[voice] ChatGPT parsing disabled ({e}); using the built-in parser')

    def _understand(self, text):
        if self.llm:
            try:
                return self.llm.parse(text)
            except Exception as e:
                print(f'[voice] ChatGPT request failed ({e}); using the built-in parser')
        return parse_command(text), None

    def feed(self, text, now=None):
        now = time.time() if now is None else now
        text = (text or '').strip()
        if not text:
            return None
        woke, rest = strip_wake_word(text)
        awake = woke or not self.require_wake or now < self.awake_until
        if not awake:
            return None
        if woke and not rest:
            self.awake_until = now + self.listen_timeout
            self.speaker.say('Yes, what would you like me to pick up?')
            return None
        cmd, reply = self._understand(rest if woke else text)
        if cmd is None:
            self.awake_until = now + self.listen_timeout
            self.speaker.say(reply or 'Sorry, which colour? Red, green, blue or yellow.')
            return None
        self.awake_until = 0.0
        if not reply:
            reply = {'stop': 'Okay, stopping.', 'start': 'Okay, starting again.', 'none': 'Okay, I will stop picking.',
                     'all': 'Okay, picking every colour.'}.get(cmd)
            if reply is None:
                parts = cmd.split()
                n = parts[0] if parts and parts[0].isdigit() else None
                cols = ' and '.join(p for p in parts if not p.isdigit()).replace('all', 'any colour')
                reply = (f'Okay, picking {n} {cols}.' if n
                         else f'Okay, picking {cols} until you tell me another colour.')
            if cmd.startswith('empty'):
                reply = 'Thank you, I will fill the bins again.'
        self.speaker.say(reply)
        self.on_command(cmd)
        return cmd

    # ------------------------------------------------------------ microphone
    def listen_forever(self, engine='auto', stop_event=None):
        """Blocking microphone loop. engine: auto | whisper | openai | google."""
        import speech_recognition as sr
        r = sr.Recognizer()
        r.dynamic_energy_threshold = True
        if engine == 'auto':
            engine = 'google'
            try:
                import whisper  # noqa: F401
                engine = 'whisper'
            except ImportError:
                if os.environ.get('OPENAI_API_KEY'):
                    engine = 'openai'
        print(f'[voice] speech-to-text engine: {engine}')
        with sr.Microphone() as mic:
            r.adjust_for_ambient_noise(mic, duration=1.0)
            print('[voice] Listening... say "Hey SCARA" (Ctrl+C to quit)', flush=True)
            while stop_event is None or not stop_event.is_set():
                try:
                    audio = r.listen(mic, timeout=5, phrase_time_limit=6)
                except sr.WaitTimeoutError:
                    continue
                try:
                    if engine == 'whisper':
                        text = r.recognize_whisper(audio, model='base.en')
                    elif engine == 'openai':
                        text = r.recognize_openai(audio)
                    else:
                        text = r.recognize_google(audio)
                except sr.UnknownValueError:
                    continue
                except Exception as e:
                    print(f'[voice] recogniser error: {e}')
                    continue
                print(f'[voice] heard: "{text}"', flush=True)
                self.feed(text)


def start_console(assistant, prompt='> '):
    """Typing instead of talking (no wake word needed)."""
    def loop():
        print('Type a colour to pick - it stays active until the next one '
              '(red / green / blue / yellow / any / none / "3 red" / stop / start):', flush=True)
        while True:
            try:
                line = input(prompt)
            except EOFError:
                return
            assistant.feed(line)
    t = threading.Thread(target=loop, daemon=True)
    t.start()
    return t
