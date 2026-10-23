// Robot adapter. Set ROBOT_URL to a robot endpoint that accepts POST {"text": "..."} and speaks it.
// Without ROBOT_URL, the page speaks via the browser's speech synthesis instead.
export const hasRobot = !!process.env.ROBOT_URL;
export async function speak(text) {
  console.log('[robot]', text);
  if (!hasRobot) return;
  try {
    await fetch(process.env.ROBOT_URL, { method: 'POST', headers: { 'content-type': 'application/json' }, body: JSON.stringify({ text }) });
  } catch (e) { console.error('robot unreachable:', e.message); }
}
