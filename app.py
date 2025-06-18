from fastapi import FastAPI, Request
import datetime

app = FastAPI()

@app.post("/flight-data")
async def receive_data(request: Request):
    payload = await request.body()
    timestamp = datetime.datetime.now().isoformat()
    print(f"[{timestamp}] Received data:\n{payload.decode()}")
    
    # Optionally: save to file
    with open("logged_data.txt", "ab") as f:
        f.write(payload + b"\n")

    return {"status": "success"}
