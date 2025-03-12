#!/usr/bin/env python3
"""
OpenAI Realtime WebRTC Single-file Example - Compact Version

Requirements: sounddevice, numpy, websockets, openai, aiohttp, pyaudio, python-dotenv, aiortc, scipy

Install: pip install sounddevice numpy websockets openai aiohttp pyaudio python-dotenv aiortc scipy

Usage:
1. Create a .env file with OPENAI_API_KEY=your-key-here
2. Run: python openai_realtime_example.py
3. Speak into your microphone
4. Press Ctrl+C to exit
"""

import asyncio
import os
import logging
import numpy as np
import sounddevice as sd
import aiohttp
import json
from collections import deque
from typing import Optional, Callable, Dict, Any, Deque, Tuple
from aiortc import RTCPeerConnection, RTCSessionDescription, MediaStreamTrack, RTCConfiguration, RTCIceServer
from aiortc.mediastreams import MediaStreamError
from av import AudioFrame
from dotenv import load_dotenv

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Audio constants
SAMPLE_RATE = 48000
CHANNELS = 1
DTYPE = np.int16
FRAME_DURATION_MS = 20
DEFAULT_CHANNELS = 2
DEFAULT_BLOCK_SIZE = int(SAMPLE_RATE * FRAME_DURATION_MS / 1000)


class AudioTrack(MediaStreamTrack):
    kind = "audio"

    def __init__(self, audio_handler):
        super().__init__()
        self._audio_handler = audio_handler
        self._queue = asyncio.Queue()
        self._task = None

    async def recv(self):
        if self._task is None:
            self._task = asyncio.create_task(self._audio_handler.start_recording(self._queue))
        try:
            return await self._queue.get()
        except Exception as e:
            logger.error(f"Error receiving audio frame: {str(e)}")
            raise MediaStreamError("Failed to receive audio frame")


class AudioHandler:
    def __init__(self, sample_rate=SAMPLE_RATE, channels=CHANNELS, frame_duration=20, 
                 dtype=DTYPE, device=None):
        self.sample_rate = sample_rate
        self.channels = channels
        self.frame_duration = frame_duration
        self.dtype = dtype
        self.device = device
        self.frame_size = int(sample_rate * frame_duration / 1000)
        self.stream = None
        self.is_recording = False
        self.is_paused = False
        self._loop = None
        self._pts = 0

    def create_audio_track(self):
        return AudioTrack(self)

    async def start_recording(self, queue):
        if self.is_recording:
            return

        self.is_recording = True
        self.is_paused = False
        self._loop = asyncio.get_running_loop()
        self._pts = 0

        try:
            def callback(indata, frames, time, status):
                if status:
                    logger.warning(f"Audio input status: {status}")
                if not self.is_paused:
                    audio_data = indata.copy()
                    if audio_data.dtype != self.dtype:
                        if self.dtype == np.int16:
                            audio_data = (audio_data * 32767).astype(self.dtype)
                        else:
                            audio_data = audio_data.astype(self.dtype)

                    frame = AudioFrame(samples=len(audio_data), layout='mono', format='s16')
                    frame.rate = self.sample_rate
                    frame.pts = self._pts
                    self._pts += len(audio_data)
                    frame.planes[0].update(audio_data.tobytes())
                    asyncio.run_coroutine_threadsafe(queue.put(frame), self._loop)

            self.stream = sd.InputStream(
                device=self.device,
                channels=self.channels,
                samplerate=self.sample_rate,
                dtype=self.dtype,
                blocksize=self.frame_size,
                callback=callback
            )
            self.stream.start()

            while self.is_recording:
                await asyncio.sleep(0.1)
        except Exception as e:
            logger.error(f"Error in audio recording: {str(e)}")
            raise
        finally:
            await self.stop()

    async def stop(self):
        self.is_recording = False
        if self.stream:
            self.stream.stop()
            self.stream.close()
            self.stream = None

    async def pause(self):
        self.is_paused = True

    async def resume(self):
        self.is_paused = False

    def set_device(self, device_id):
        self.device = device_id
        if self.stream:
            self.stream.stop()
            self.stream.close()
            self.stream = None


class AudioOutput:
    def __init__(self, sample_rate=SAMPLE_RATE, channels=DEFAULT_CHANNELS, 
                 dtype=DTYPE, device=None, buffer_size=50):
        self.sample_rate = sample_rate
        self.channels = channels
        self.dtype = dtype
        self.device = device
        self.block_size = DEFAULT_BLOCK_SIZE
        self._buffer = deque(maxlen=buffer_size)
        self._queue = asyncio.Queue()
        self.stream = None
        self.is_playing = False
        self._task = None
        self._remaining_data = None

    async def start(self):
        if self.is_playing:
            return

        try:
            self.stream = sd.OutputStream(
                device=self.device,
                samplerate=self.sample_rate,
                channels=self.channels,
                dtype=self.dtype,
                blocksize=self.block_size,
                callback=self._audio_callback,
                prime_output_buffers_using_stream_callback=True
            )
            self.stream.start()
            self.is_playing = True
            self._task = asyncio.create_task(self._process_audio())
        except Exception as e:
            logger.error(f"Failed to start audio output: {str(e)}")
            raise

    async def stop(self):
        if not self.is_playing:
            return
            
        try:
            self.is_playing = False
            if self._task:
                self._task.cancel()
                self._task = None

            if self.stream:
                self.stream.stop()
                self.stream.close()
                self.stream = None

            self._buffer.clear()
            while not self._queue.empty():
                try:
                    self._queue.get_nowait()
                except asyncio.QueueEmpty:
                    break

            logger.info("Audio output stopped")
        except Exception as e:
            logger.error(f"Error stopping audio output: {str(e)}")

    def _audio_callback(self, outdata, frames, time, status):
        if status:
            logger.warning(f"Audio output status: {status}")

        try:
            if len(self._buffer) > 0:
                data = self._buffer.popleft()
                outdata[:] = data.reshape(outdata.shape)
            else:
                outdata.fill(0)
        except Exception as e:
            logger.error(f"Error in audio callback: {str(e)}")
            outdata.fill(0)

    async def _process_audio(self):
        try:
            while self.is_playing:
                if len(self._buffer) < self._buffer.maxlen:
                    try:
                        data = await asyncio.wait_for(self._queue.get(), 0.1)
                        self._buffer.append(data)
                    except (asyncio.TimeoutError, asyncio.QueueEmpty):
                        continue
                    except Exception as e:
                        logger.error(f"Error getting data from queue: {str(e)}")
                else:
                    await asyncio.sleep(0.001)
        except asyncio.CancelledError:
            logger.info("Audio processing task cancelled")
        except Exception as e:
            logger.error(f"Error processing audio: {str(e)}")

    async def play_frame(self, frame):
        try:
            audio_data = frame.to_ndarray()
            if audio_data.dtype != self.dtype:
                audio_data = audio_data.astype(self.dtype)

            if self._queue.qsize() >= self._buffer.maxlen:
                try:
                    self._queue.get_nowait()
                except asyncio.QueueEmpty:
                    pass

            await self._queue.put(audio_data)
        except Exception as e:
            logger.error(f"Error queueing audio frame: {str(e)}")
            raise


class WebRTCManager:
    OPENAI_API_BASE = "https://api.openai.com/v1"
    REALTIME_SESSION_URL = f"{OPENAI_API_BASE}/realtime/sessions"
    REALTIME_URL = f"{OPENAI_API_BASE}/realtime"

    def __init__(self):
        self.ice_servers = [RTCIceServer(urls=["stun:stun.l.google.com:19302"])]
        self.audio_output = None
        self.peer_connection = None
        self.output_device = None

    async def create_connection(self):
        config = RTCConfiguration(iceServers=self.ice_servers)
        self.peer_connection = RTCPeerConnection(config)

        self.audio_output = AudioOutput(device=self.output_device)
        await self.audio_output.start()

        @self.peer_connection.on("track")
        async def on_track(track):
            logger.info(f"Received {track.kind} track from remote")
            if track.kind == "audio":
                @track.on("ended")
                async def on_ended():
                    if self.audio_output:
                        await self.audio_output.stop()

                while True:
                    try:
                        frame = await track.recv()
                        if self.audio_output and frame:
                            await self.audio_output.play_frame(frame)
                    except Exception as e:
                        logger.error(f"Error processing remote audio frame: {str(e)}")
                        break

        @self.peer_connection.on("connectionstatechange")
        async def on_connectionstatechange():
            logger.info(f"Connection state changed to: {self.peer_connection.connectionState}")
            if self.peer_connection.connectionState == "failed" and self.audio_output:
                await self.audio_output.stop()

        @self.peer_connection.on("iceconnectionstatechange")
        async def on_iceconnectionstatechange():
            logger.info(f"ICE connection state changed to: {self.peer_connection.iceConnectionState}")

        return self.peer_connection

    async def cleanup(self):
        if self.audio_output:
            await self.audio_output.stop()
            self.audio_output = None

        if self.peer_connection:
            await self.peer_connection.close()
            self.peer_connection = None

    async def get_ephemeral_token(self, api_key, model, system_prompt=None):
        headers = {
            "Authorization": f"Bearer {api_key}",
            "Content-Type": "application/json"
        }

        data = {"model": model, "voice": "alloy"}
        if system_prompt:
            data["instructions"] = system_prompt

        try:
            async with aiohttp.ClientSession() as session:
                async with session.post(self.REALTIME_SESSION_URL, headers=headers, json=data) as response:
                    if response.status not in [200, 201]:
                        error_text = await response.text()
                        raise Exception(f"Failed to get ephemeral token: {error_text}")

                    result = await response.json()
                    return result["client_secret"]["value"]
        except Exception as e:
            logger.error(f"Failed to get ephemeral token: {str(e)}")
            raise

    async def connect_to_openai(self, api_key, model, offer, system_prompt=None):
        try:
            ephemeral_token = await self.get_ephemeral_token(api_key, model, system_prompt)
            headers = {
                "Authorization": f"Bearer {ephemeral_token}",
                "Content-Type": "application/sdp"
            }

            async with aiohttp.ClientSession() as session:
                async with session.post(f"{self.REALTIME_URL}?model={model}", 
                                        headers=headers, data=offer.sdp) as response:
                    if response.status not in [200, 201]:
                        error_text = await response.text()
                        raise Exception(f"OpenAI WebRTC error: {error_text}")

                    sdp_answer = await response.text()
                    return {"type": "answer", "sdp": sdp_answer}
        except Exception as e:
            logger.error(f"Failed to connect to OpenAI: {str(e)}")
            raise

    async def handle_ice_candidate(self, peer_connection, candidate):
        try:
            await peer_connection.addIceCandidate(candidate)
        except Exception as e:
            logger.error(f"Error adding ICE candidate: {str(e)}")
            raise


class OpenAIWebRTCClient:
    def __init__(self, api_key, model="gpt-4o-realtime-preview", sample_rate=SAMPLE_RATE, 
                 channels=CHANNELS, frame_duration=FRAME_DURATION_MS, 
                 input_device=None, output_device=None, system_prompt=None):
        self.api_key = api_key
        self.model = model
        self.system_prompt = system_prompt
        self.audio_handler = AudioHandler(
            sample_rate=sample_rate,
            channels=channels,
            frame_duration=frame_duration,
            device=input_device
        )
        self.webrtc_manager = WebRTCManager()
        if output_device is not None:
            self.webrtc_manager.output_device = output_device

        self.peer_connection = None
        self.is_streaming = False
        self.on_transcription = None

    async def start_streaming(self):
        if self.is_streaming:
            logger.warning("Streaming is already active")
            return

        try:
            self.peer_connection = await self.webrtc_manager.create_connection()
            audio_track = self.audio_handler.create_audio_track()
            self.peer_connection.addTransceiver(audio_track, "sendrecv")

            offer = await self.peer_connection.createOffer()
            await self.peer_connection.setLocalDescription(offer)

            response = await self.webrtc_manager.connect_to_openai(
                self.api_key, self.model, offer, self.system_prompt
            )

            answer = RTCSessionDescription(sdp=response["sdp"], type=response["type"])
            await self.peer_connection.setRemoteDescription(answer)

            self.is_streaming = True
            logger.info("Streaming started successfully")
        except Exception as e:
            logger.error(f"Failed to start streaming: {str(e)}")
            await self.stop_streaming()
            raise

    async def stop_streaming(self):
        if not self.is_streaming:
            return

        try:
            await self.webrtc_manager.cleanup()
            await self.audio_handler.stop()
            self.is_streaming = False
            logger.info("Streaming stopped successfully")
        except Exception as e:
            logger.error(f"Error while stopping streaming: {str(e)}")
            raise

    async def pause_streaming(self):
        if self.is_streaming:
            await self.audio_handler.pause()

    async def resume_streaming(self):
        if self.is_streaming:
            await self.audio_handler.resume()

    def set_audio_device(self, device_id):
        self.audio_handler.set_device(device_id)

    def _handle_transcription(self, text):
        if self.on_transcription:
            self.on_transcription(text)


async def main():
    api_key = os.getenv("OPENAI_API_KEY")
    if not api_key:
        print("Error: OPENAI_API_KEY environment variable not set.")
        print("Please create a .env file with your OpenAI API key.")
        return

    client = OpenAIWebRTCClient(
        api_key=api_key,
        model="gpt-4o-realtime-preview",
        system_prompt="You are providing me help around the University of Waterloo campus. You are the help desk, try your best to help students and staff with their questions."
    )

    def on_transcription(text):
        print(f"Transcription: {text}")

    client.on_transcription = on_transcription

    try:
        print("Starting streaming session...")
        await client.start_streaming()
        print("Streaming started. Speak into your microphone.")
        print("Press Ctrl+C to stop.")
        
        while True:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping streaming...")
    finally:
        await client.stop_streaming()
        print("Streaming stopped.")


if __name__ == "__main__":
    load_dotenv()
    asyncio.run(main()) 