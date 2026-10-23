import express from 'express';
import { createServer } from 'http';
import { WebSocketServer } from 'ws';
import Anthropic from '@anthropic-ai/sdk';
import { speak, command, hasRobot } from './robot.js';

const MODEL = process.env.MODEL || 'claude-sonnet-5-5';
const client = new Anthropic(); // needs ANTHROPIC_API_KEY
const app = express();
app.use(express.static('public'));
app.use('/three', express.static('node_modules/three'));
const server = createServer(app);
const wss = new WebSocketServer({ server, path: '/ws' });

const SYSTEM = process.env.TEACHER_PROMPT || `You are a friendly robot science teacher (you have a body: you can walk, turn, wave and kick the ball) for kids aged 8-12, standing next to a 3D ball-kicking simulator.
Teach simple physics (speed, velocity, acceleration, gravity, projectile motion, kinematics, air resistance, spin) by running experiments with your tools, then explaining what happened.
Rules: keep replies to 2-4 short sentences (they are spoken aloud); no markdown, no emoji; use simple words; use real numbers from tool results; run one experiment at a time, ask the kid to predict before kicking when it makes sense; compare against the ideal_no_air_prediction when relevant.`;

const tools = [
  { name: 'kick', description: 'The robot walks behind the ball, kicks it, and waits until it first lands. Returns measured range, hang time, peak height, and the ideal no-air prediction.',
    input_schema: { type: 'object', properties: {
      speed: { type: 'number', description: 'launch speed m/s (1-40)' }, angle: { type: 'number', description: 'launch angle above ground, degrees (0-90)' },
      azimuth: { type: 'number', description: 'left/right aim, degrees (-45..45)' }, backspin: { type: 'number', description: 'rev/s, negative = topspin' },
      sidespin: { type: 'number', description: 'rev/s, curves the ball sideways' }, drag: { type: 'boolean', description: 'air resistance on/off for this kick' } },
      required: ['speed', 'angle'] } },
  { name: 'walk_to', description: 'Walk the robot to a spot on the field. Metres: x is forward (the ball starts at 0,0), z is left/right.',
    input_schema: { type: 'object', properties: { x: { type: 'number' }, z: { type: 'number' } }, required: ['x', 'z'] } },
  { name: 'turn_to', description: 'Turn the robot to face a direction in degrees (0 = forward, positive = right).',
    input_schema: { type: 'object', properties: { degrees: { type: 'number' } }, required: ['degrees'] } },
  { name: 'wave', description: 'Robot waves hello.', input_schema: { type: 'object', properties: {} } },
  { name: 'set_physics', description: 'Change the world. Use to compare e.g. Moon gravity (1.62) vs Earth (9.81).',
    input_schema: { type: 'object', properties: { gravity: { type: 'number' }, air_resistance: { type: 'boolean' }, bounciness: { type: 'number', description: '0-1' }, magnus_spin_effect: { type: 'boolean' } } } },
  { name: 'get_state', description: 'Current ball time, position, velocity, and settings.', input_schema: { type: 'object', properties: {} } },
  { name: 'reset', description: 'Put the ball back at the start.', input_schema: { type: 'object', properties: {} } },
];

let browser = null;
const waiting = new Map();
const broadcast = (o) => browser?.readyState === 1 && browser.send(JSON.stringify(o));
const callBrowser = (name, input) => new Promise((resolve, reject) => {
  if (browser?.readyState !== 1) return reject(new Error('simulator page is not connected'));
  const id = Math.random().toString(36).slice(2);
  const t = setTimeout(() => { waiting.delete(id); reject(new Error('timed out waiting for simulator')); }, 30000);
  waiting.set(id, (r) => { clearTimeout(t); resolve(r); });
  broadcast({ type: 'tool', id, name, input });
});

const history = [];
let busy = Promise.resolve();
async function chat(text) {
  history.push({ role: 'user', content: text });
  for (let turn = 0; turn < 8; turn++) {
    const res = await client.messages.create({ model: MODEL, max_tokens: 1024, system: SYSTEM, tools, messages: history });
    history.push({ role: 'assistant', content: res.content });
    const said = res.content.filter((b) => b.type === 'text').map((b) => b.text).join(' ').trim();
    if (said) { broadcast({ type: 'say', text: said, speak: !hasRobot }); await speak(said); }
    if (res.stop_reason !== 'tool_use') return;
    const results = [];
    for (const b of res.content.filter((c) => c.type === 'tool_use')) {
      let out, err = false;
      try { [out] = await Promise.all([callBrowser(b.name, b.input), command(b.name, b.input)]); } catch (e) { out = { error: e.message }; err = true; }
      results.push({ type: 'tool_result', tool_use_id: b.id, content: JSON.stringify(out), is_error: err });
    }
    history.push({ role: 'user', content: results });
  }
}

wss.on('connection', (ws) => {
  browser = ws;
  ws.on('message', (raw) => {
    const m = JSON.parse(raw);
    if (m.type === 'result') waiting.get(m.id)?.(m.result), waiting.delete(m.id);
    if (m.type === 'chat') busy = busy.then(() => chat(m.text)).catch((e) => { console.error(e); broadcast({ type: 'say', text: 'Oops, my brain hiccuped: ' + e.message }); });
  });
});

server.listen(process.env.PORT || 3000, () => console.log(`open http://localhost:${process.env.PORT || 3000}  (robot: ${hasRobot ? process.env.ROBOT_URL : 'browser speech'})`));
