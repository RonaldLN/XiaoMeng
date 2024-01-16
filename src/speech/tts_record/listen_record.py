#!/usr/bin/env python
# -*- coding: utf-8 -*-
from speech_recognition import *
from pydub import AudioSegment#音频预处理
from pydub.playback import play#用于播放音频
m = Microphone(sample_rate=16000, chunk_size=1024)#获取麦克风
xm=Recognizer()
with m as s:
    print("=====Please wait a minute...")
    xm.adjust_for_ambient_noise(m,2)#检测噪声
    record=AudioSegment.from_wav("/home/kenny/Desktop/ws_XfSpeech/tts_record/cue_tone.wav")
    play(record)
    print("+++++Now start to listen...")
    audio = xm.listen(m, None, 10, None)#录制音频
    print("=====Got it!Now start to recognize...")
raw=audio.get_raw_data(None,None)#获取原始音频数据
wav_file="/home/kenny/Desktop/ws_XfSpeech/tts_record/gpsr_questions/trial.wav"#原始音频存储路径
#如果有必要可以微调音频格式再存储
wav_writer = wave.open(wav_file, "wb")
wav_writer.setframerate(audio.sample_rate)
wav_writer.setsampwidth(audio.sample_width)
wav_writer.setnchannels(1)
wav_writer.writeframes(raw)#对原始音频数据做微调
wav_writer.close()