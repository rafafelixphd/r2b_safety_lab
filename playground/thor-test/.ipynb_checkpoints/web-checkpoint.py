import io
import asyncio
from fastapi import FastAPI, Response
from fastapi.responses import StreamingResponse, HTMLResponse
from ai2thor.controller import Controller
from PIL import Image
from pyvirtualdisplay import Display

# Start Virtual Display for the server
v_display = Display(visible=0, size=(1024, 768))
v_display.start()

app = FastAPI()
# Initialize AI2-THOR
controller = Controller(scene="FloorPlan1", width=640, height=480)

def get_frame():
    """Converts the current AI2-THOR frame to JPEG bytes."""
    img = Image.fromarray(controller.last_event.frame)
    buf = io.BytesIO()
    img.save(buf, format='JPEG')
    return buf.getvalue()

@app.get("/video_feed")
async def video_feed():
    """Streams the robot camera as an MJPEG stream."""
    async def frame_generator():
        while True:
            frame = get_frame()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            await asyncio.sleep(0.05)  # ~20 FPS

    return StreamingResponse(frame_generator(), media_type="multipart/x-mixed-replace; boundary=frame")

@app.post("/action/{action_name}")
async def take_action(action_name: str):
    """Maps API calls to AI2-THOR actions."""
    mapping = {
        "w": "MoveAhead",
        "s": "MoveBack",
        "a": "RotateLeft",
        "d": "RotateRight"
    }
    thor_action = mapping.get(action_name.lower())
    if thor_action:
        controller.step(action=thor_action)
    return {"status": "success", "action": thor_action}

@app.get("/", response_class=HTMLResponse)
async def index():
    """A simple HTML page to show the video and catch keystrokes."""
    return """
    <html>
        <body style="background: #222; color: white; text-align: center;">
            <h1>AI2-THOR Remote Control</h1>
            <img src="/video_feed" style="border: 2px solid #555;">
            <p>Use <b>WASD</b> keys to move the robot.</p>
            <script>
                document.addEventListener('keydown', (e) => {
                    const key = e.key.toLowerCase();
                    if (['w', 'a', 's', 'd'].includes(key)) {
                        fetch(`/action/${key}`, {method: 'POST'});
                    }
                });
            </script>
        </body>
    </html>
    """