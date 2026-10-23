# Kick Lab

three.js ball-kicking physics sim. A simulated humanoid robot (rigid-body physics: mass, traction-limited
acceleration, torque-limited turning, motorised kicking leg) is driven by a Claude teacher agent.

    npm install
    ANTHROPIC_API_KEY=... npm start     # PowerShell: $env:ANTHROPIC_API_KEY="..."; npm start
    open http://localhost:3000

- `public/physics.js`  ball: gravity, drag, Magnus spin force, bounce, friction  (`npm test`)
- `public/robot-sim.js` robot body + leg dynamics (`node test-robot.js`)
- `server.js` Claude agent + tools; `robot.js` optional bridge to a physical robot (`ROBOT_URL`)
