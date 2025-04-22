import time
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from flask import Flask, request, render_template_string, redirect, url_for

# --- Configuration ---
SERVO_FREQUENCY = 50
SERVO_CHANNELS = [0, 1, 2, 3, 4, 5]
STEP_DELAY = 0.05     # Delay after each global step
STEP_SIZE = 1         # Degrees per sub‑step

# --- Initialization of I2C and PCA9685 ---
print("Initializing I2C...")
i2c = busio.I2C(board.SCL, board.SDA)
print("Initializing PCA9685...")
pca = PCA9685(i2c)
pca.frequency = SERVO_FREQUENCY

# --- Create Servo Objects and state ---
servos = {}
current_angles = {}
for ch in SERVO_CHANNELS:
    s = servo.Servo(pca.channels[ch])
    servos[ch] = s
    current_angles[ch] = 90
    s.angle = 90

# --- Smoothing Functions ---
def smooth_move(servo_obj, start, end, step_size, delay):
    step = step_size if end >= start else -step_size
    for a in range(int(start), int(end), int(step)):
        servo_obj.angle = a
        time.sleep(delay)
    servo_obj.angle = end


def smooth_move_multi(channels, targets, step_size, delay):
    # compute per-servo delta and direction
    deltas = []
    for ch, tgt in zip(channels, targets):
        delta = tgt - current_angles[ch]
        direction = step_size if delta >= 0 else -step_size
        deltas.append((delta, direction))

    # how many sub-steps we need
    steps = int(max(abs(d) for d, _ in deltas) / step_size)

    # step through motions
    for _ in range(steps):
        for idx, ch in enumerate(channels):
            delta, direction = deltas[idx]
            if abs(current_angles[ch] - (current_angles[ch] + delta)) >= step_size:
                current_angles[ch] += direction
                servos[ch].angle = current_angles[ch]
                time.sleep(delay)

    # ensure exact final positions
    for ch, tgt in zip(channels, targets):
        servos[ch].angle = tgt
        current_angles[ch] = tgt

# --- Flask App ---
app = Flask(__name__)

HTML_TEMPLATE = '''
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>Servo Controller</title>
    <style>
      body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background: #f4f6f8; color: #333; padding: 20px; }
      h1 { text-align: center; margin-bottom: 30px; }
      .controls { display: flex; flex-direction: column; align-items: center; gap: 20px; }
      .servo-control { background: #fff; border-radius: 12px; box-shadow: 0 2px 6px rgba(0,0,0,0.1); padding: 15px; width: 240px; text-align: center; }
      .servo-control label { font-weight: bold; display: block; margin-bottom: 10px; }
      .gauge { width: 200px; height: 100px; cursor: pointer; }
      input[type=number] { width: 60px; font-size: 1rem; text-align: center; margin-top: 10px; }
      button { margin-top: 30px; padding: 10px 20px; font-size: 1rem; border: none; border-radius: 8px; background: #007bff; color: white; cursor: pointer; }
      button:hover { background: #0056b3; }
    </style>
</head>
<body>
  <h1>Servo Controller</h1>
  <form method="POST" id="servoForm">
    <div class="controls">
    {% for ch in channels %}
      <div class="servo-control">
        <label>Channel {{ch}}: <span id="lbl{{ch}}">{{angles[ch]}}</span>°</label>
        <svg class="gauge" id="gauge{{ch}}" viewBox="0 0 200 100">
          <!-- semicircle arc -->
          <path d="M10,100 A90,90 0 0,1 190,100" fill="none" stroke="#ddd" stroke-width="10"/>
          <!-- pointer starts pointing left -->
          <line id="ptr{{ch}}" x1="100" y1="100" x2="10" y2="100" stroke="#007bff" stroke-width="6" stroke-linecap="round" transform="rotate({{angles[ch]}},100,100)"/>
        </svg>
        <input type="number" id="num{{ch}}" name="servo{{ch}}" min="0" max="180" value="{{angles[ch]}}" />
      </div>
    {% endfor %}
    </div>
    <div style="text-align:center;"><button type="submit">Apply</button></div>
  </form>
  <script>
    let draggingCh = null;
    function updateGauge(ch, angle) {
      const ptr = document.getElementById('ptr'+ch);
      const lbl = document.getElementById('lbl'+ch);
      const num = document.getElementById('num'+ch);
      angle = Math.max(0, Math.min(180, angle));
      ptr.setAttribute('transform', `rotate(${angle},100,100)`);
      lbl.textContent = angle.toFixed(0);
      num.value = angle.toFixed(0);
    }
    function startDrag(event) {
      const svg = event.currentTarget;
      draggingCh = svg.id.replace('gauge','');
      document.addEventListener('mousemove', onDrag);
      document.addEventListener('mouseup', endDrag);
      onDrag(event);
    }
    function onDrag(event) {
      if (draggingCh === null) return;
      const svg = document.getElementById('gauge'+draggingCh);
      const pt = svg.createSVGPoint();
      pt.x = event.clientX;
      pt.y = event.clientY;
      const svgP = pt.matrixTransform(svg.getScreenCTM().inverse());
      const cx = 100, cy = 100;
      const dx = svgP.x - cx, dy = cy - svgP.y;
      let raw = Math.atan2(dy, dx) * 180/Math.PI;
      let angle = 180 - raw;
      updateGauge(draggingCh, angle);
    }
    function endDrag() {
      draggingCh = null;
      document.removeEventListener('mousemove', onDrag);
      document.removeEventListener('mouseup', endDrag);
    }
    document.addEventListener('DOMContentLoaded', ()=>{
      {% for ch in channels %}
        const num{{ch}} = document.getElementById('num{{ch}}');
        num{{ch}}.addEventListener('input', ()=> updateGauge({{ch}}, parseFloat(num{{ch}}.value)));
        document.getElementById('gauge{{ch}}').addEventListener('mousedown', startDrag);
      {% endfor %}
    });
  </script>
</body>
</html>
'''

@app.route('/', methods=['GET', 'POST'])
def index():
    if request.method == 'POST':
        # parse target angles
        targets = {}
        for ch in SERVO_CHANNELS:
            key = f'servo{ch}'
            try:
                targets[ch] = float(request.form.get(key, current_angles[ch]))
            except ValueError:
                targets[ch] = current_angles[ch]
        # find channels that need moving
        move_chs = [ch for ch in SERVO_CHANNELS if abs(targets[ch] - current_angles[ch]) >= 0.5]
        if move_chs:
            move_tgts = [targets[ch] for ch in move_chs]
            smooth_move_multi(move_chs, move_tgts, STEP_SIZE, STEP_DELAY)
        return redirect(url_for('index'))

    # GET: render with current angles
    return render_template_string(
        HTML_TEMPLATE,
        channels=SERVO_CHANNELS,
        angles=current_angles
    )

if __name__ == '__main__':
    print("Starting Flask server on http://0.0.0.0:5000")
    app.run(host='0.0.0.0', port=5000)
