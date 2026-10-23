// Real-robot adapter. Set ROBOT_URL to your robot's HTTP endpoint (e.g. http://192.168.1.50:8080).
// The agent's actions are POSTed there as JSON:
//   {"action":"say","text":"..."}            speak aloud
//   {"action":"walk_to","x":-0.7,"z":0}      walk (metres; x forward, ball start is 0,0)
//   {"action":"turn_to","degrees":30}        face a heading
//   {"action":"wave"}
//   {"action":"kick","power":0.6,"loft":25,"azimuth":0,"backspin":0,"sidespin":0}   (power = fraction of leg motor torque)
// Wire these to your robot's SDK in a small bridge, or edit command() below to call the SDK directly.
// The 3D robot in the page always mirrors the same commands, so it works with no hardware too.
export const hasRobot = !!process.env.ROBOT_URL;
const post = async (body) => {
  if (!hasRobot) return;
  try {
    await fetch(process.env.ROBOT_URL, { method: 'POST', headers: { 'content-type': 'application/json' }, body: JSON.stringify(body) });
  } catch (e) { console.error('robot unreachable:', e.message); }
};
export async function speak(text) { console.log('[robot says]', text); await post({ action: 'say', text }); }
export async function command(action, input = {}) {
  if (['walk_to', 'turn_to', 'wave', 'kick'].includes(action)) { console.log('[robot do]', action, JSON.stringify(input)); await post({ action, ...input }); }
}
