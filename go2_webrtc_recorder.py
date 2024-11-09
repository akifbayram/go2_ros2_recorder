import asyncio
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from aiortc.contrib.media import MediaRecorder
import aiohttp
import logging
import json
import hashlib
import base64

ROBOT_IP = "192.168.1.100"
ROBOT_TOKEN = "your_token_here" 

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

class Go2WebRTCConnection:
    def __init__(self, ip, token):
        self.pc = RTCPeerConnection()
        self.ip = ip
        self.token = token

        self.video_track = None
        self.recorder = MediaRecorder('output.mp4')  
        
        self.pc.on("track", self.on_track)
        self.pc.on("connectionstatechange", self.on_connection_state_change)

        self.data_channel = self.pc.createDataChannel("data", id=2)
        self.data_channel.on("open", self.on_data_channel_open)
        self.data_channel.on("message", self.on_data_channel_message)

    def on_track(self, track):
        if track.kind == "video":
            logger.info("Video track received, starting recording")
            self.recorder.addTrack(track)

    def on_connection_state_change(self):
        logger.info(f"Connection state changed: {self.pc.connectionState}")
        if self.pc.connectionState == "failed":
            logger.error("Connection failed")

    def on_data_channel_open(self):
        logger.info("Data channel is open")

    def on_data_channel_message(self, message):
        logger.info(f"Received message: {message}")

    async def generate_offer(self):
        # Generate and set local offer SDP
        offer = await self.pc.createOffer()
        await self.pc.setLocalDescription(offer)
        return offer.sdp

    async def connect_to_robot(self):
        offer_sdp = await self.generate_offer()

        async with aiohttp.ClientSession() as session:
            url = f"http://{self.ip}:8081/offer"
            headers = {"Content-Type": "application/json"}
            data = {
                "sdp": offer_sdp,
                "id": "STA_localNetwork",
                "type": "offer",
                "token": self.token,
            }

            async with session.post(url, json=data, headers=headers) as resp:
                if resp.status == 200:
                    answer_data = await resp.json()
                    logger.info("Received answer from server")
                    answer_sdp = answer_data.get("sdp")
                    await self.set_remote_answer(answer_sdp)
                    await self.start_recording()

    async def set_remote_answer(self, sdp):
        answer = RTCSessionDescription(sdp, type="answer")
        await self.pc.setRemoteDescription(answer)

    async def start_recording(self):
        logger.info("Starting recording...")
        await self.recorder.start()

    async def stop_recording(self):
        logger.info("Stopping recording...")
        await self.recorder.stop()
        await self.pc.close()

    async def send_command(self, command):
        """Send a command via the data channel."""
        if self.data_channel.readyState == "open":
            self.data_channel.send(command)
            logger.info(f"Sent command: {command}")
        else:
            logger.error("Data channel is not open. Command not sent.")

async def main():
    connection = Go2WebRTCConnection(ip=ROBOT_IP, token=ROBOT_TOKEN)
    await connection.connect_to_robot()

    # Example of sending a command
    await connection.send_command("Your command here")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("Recording interrupted")

