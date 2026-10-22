/* Voice commands.
 *   Android app : the phone's own speech recogniser (window.ScaraNative, works offline when
 *                 the phone has offline speech installed - Google app > Settings > Voice).
 *   Browser     : Chrome / Edge Web Speech API (needs internet, and http://localhost or https).
 * listen(onText, onState) -> starts one recognition; onText(text) gets what was heard.
 *
 * parse(text) turns a sentence into app actions: colours, sequences ("red then blue"),
 * counts ("3 green"), "any", "stop picking", start / stop the belt, open / close / home,
 * and the world: "sorting", "command", "table".
 */
(() => {
'use strict';

const FIX = {                         // what speech recognisers often write for the colour names
  read: 'red', rid: 'red', bread: 'red', fred: 'red', ret: 'red',
  grin: 'green', greene: 'green', queen: 'green', grain: 'green',
  blew: 'blue', blu: 'blue', glue: 'blue', bloo: 'blue',
  yello: 'yellow', jello: 'yellow', hello: 'yellow', yelo: 'yellow',
  one: '1', two: '2', to: 'to', too: '2', three: '3', four: '4', for: 'for', five: '5', six: '6',
  colour: 'color', colours: 'color', colors: 'color',
};
const COLORS = ['red', 'green', 'blue', 'yellow'];

function normalise(text) {
  return String(text || '').toLowerCase().replace(/[.,!?;:]/g, ' ').split(/\s+/).filter(Boolean)
    .map((w) => FIX[w] || w);
}

/* -> list of actions: {kind: 'pick'|'belt'|'gripper'|'home'|'world'|'estop', ...} */
function parse(text) {
  const w = normalise(text), has = (...k) => k.some((x) => w.includes(x));
  const s = ' ' + w.join(' ') + ' ';
  if (!w.length) return [];
  if (has('emergency') || s.includes(' stop everything ') || s.includes(' stop robot ') || s.includes(' freeze ')) return [{ kind: 'estop' }];
  if (has('resume', 'unlock') || s.includes(' release stop ') || s.includes(' release the stop ')) return [{ kind: 'release' }];
  if (has('sorting', 'sort')) return [{ kind: 'world', world: 'sort' }];
  if (s.includes(' command mode ') || s.includes(' command cell ')) return [{ kind: 'world', world: 'command' }];
  if (has('table', 'manual')) return [{ kind: 'world', world: 'manual' }];
  if (s.includes(' stop picking ') || has('nothing', 'none')) return [{ kind: 'pick', text: 'none' }];
  const cols = w.filter((x) => COLORS.includes(x));
  if (cols.length) return [{ kind: 'pick', text: w.filter((x) => COLORS.includes(x) || /^\d+$/.test(x) || x === 'then').join(' ') }];
  if (has('any', 'every', 'everything') || s.includes(' all color')) return [{ kind: 'pick', text: 'any' }];
  if (has('open', 'release', 'drop') || s.includes(' let go ')) return [{ kind: 'gripper', close: false }];
  if (has('close', 'grab', 'grip', 'catch', 'hold')) return [{ kind: 'gripper', close: true }];
  if (has('home', 'park', 'ready')) return [{ kind: 'home' }];
  if (has('start', 'run', 'go', 'belt', 'conveyor') && !has('stop', 'halt', 'pause')) return [{ kind: 'belt', run: true }];
  if (has('stop', 'halt', 'pause')) return [{ kind: 'belt', run: false }];
  return [];
}

let busy = false;
function available() {
  return !!((window.ScaraNative && window.ScaraNative.voiceAvailable && window.ScaraNative.voiceAvailable())
    || window.SpeechRecognition || window.webkitSpeechRecognition);
}

function listen(onText, onState) {
  onState = onState || (() => {});
  if (busy) return;
  if (window.ScaraNative && window.ScaraNative.startVoice) {
    busy = true; onState('listening');
    window.onNativeVoice = (t, err) => {
      busy = false; onState('idle');
      if (t) onText(t);
      else onState(err === 'no-recognizer' ? 'no-recognizer' : 'nothing');
    };
    try { window.ScaraNative.startVoice(); } catch (e) { busy = false; onState('error'); }
    return;
  }
  const SR = window.SpeechRecognition || window.webkitSpeechRecognition;
  if (!SR) { onState('unsupported'); return; }
  const r = new SR();
  r.lang = navigator.language || 'en-US';
  r.interimResults = false; r.maxAlternatives = 3;
  busy = true; onState('listening');
  let heard = false, failed = false;
  r.onresult = (e) => { heard = true; onText(e.results[0][0].transcript); };
  r.onerror = (e) => { failed = true; busy = false; onState(e.error === 'not-allowed' || e.error === 'service-not-allowed' ? 'denied' : (e.error === 'network' ? 'network' : 'nothing')); };
  r.onend = () => { busy = false; onState(heard || failed ? 'idle' : 'nothing'); };
  try { r.start(); } catch (e) { busy = false; onState('error'); return; }
  setTimeout(() => { if (busy) { try { r.abort(); } catch (e) { /* gone */ } busy = false; onState('nothing'); } }, 12000);
}

window.ScaraVoice = { listen, parse, available, normalise };
})();
