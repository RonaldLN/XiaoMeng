import sys
import struct
import wave
from threading import Thread

import pvcobra
from pvrecorder import PvRecorder
import time


class CobraVAD(Thread):
    """
    用于麦克风获取音频的眼镜蛇人声活动监测引擎
    https://picovoice.ai/
    本文件来自于对cobra_demo_mic.py的修改，https://github.com/Picovoice/cobra/blob/main/demo/python/cobra_demo_mic.py
    将命令行调用改成了传递参数，删除了部分代码
    需要access_key，我的为"CWwRg5ghpE5IvLl/xqJNHoW6snSeLHvEuS4Us17YphjYW8esgcUFXA==",不需要替换
    """

    def __init__(
            self,
            library_path=pvcobra.LIBRARY_PATH,
            access_key="CWwRg5ghpE5IvLl/xqJNHoW6snSeLHvEuS4Us17YphjYW8esgcUFXA==",
            output_path='vad.wav',
            input_device_index=-1):
        """
        library_path为Cobra动态库的绝对路径，不要修改
        access_key需要从picovoice官网获得，这里填的是我的，不要修改
        output_path为音频文件储存路径
        input_device_index为麦克风编号，有可能需要改，默认为正在使用的那个
        """

        super(CobraVAD, self).__init__()

        self._library_path = library_path
        self._access_key = access_key
        self._input_device_index = input_device_index
        self._output_path = output_path

    def run(self, mode='right_now', sensitive=80):
        """
        重载自Thread的函数，将会启用新线程
        创建一个音频输入流，创建一个Cobra实例，监听音频输入流，并且以某种规则储存
        mode为识别规则：有right_now和wait_human_voice两种魔术
            right_now采用立即开始，在监测到有人说过话了，且刚刚两秒无人说话后停止录音
            wait_human_voice可以采用从第一次监测到有人说话开始
        sensitive可以指定超过sensitive/100，即判断音频帧为人声的下限概率
        """
        # Cobra空引用
        cobra = None
        # 音频接受器空引用
        recorder = None
        # 音频文件储存器空引用
        wav_file = None

        try:
            # 实例化pvcobra
            cobra = pvcobra.create(
                library_path=self._library_path, access_key=self._access_key)
            # 实例化音频接收器
            recorder = PvRecorder(device_index=self._input_device_index, frame_length=512)
            # 启用音频接收器
            recorder.start()
            # 如果给出了储存路径
            wav_file = wave.open(self._output_path, "w")
            wav_file.setparams((1, 2, 16000, 512, "NONE", "NONE"))
            print("Listening...")
            # begin_flag表示可以开始录音
            # human_voice_time储存前一次监测到人声的时间
            # not_human_voice_time储存前一次监测到非人声的时间

            if mode == 'right_now':
                begin_flag = True
            else:
                begin_flag = False
            if mode == 'wait_human_voice':
                begin_flag = False
            human_voice_time = None
            not_human_voice_time = None

            while True:
                # pcm为音频帧
                pcm = recorder.read()
                # 将音频帧写入音频文件
                if begin_flag:
                    wav_file.writeframes(struct.pack("h" * len(pcm), *pcm))
                # 识别音频帧，判断为人声的概率
                voice_probability = cobra.process(pcm)
                percentage = voice_probability * 100
                # 在终端以条状框的形式打印结果
                bar_length = int((percentage / 10) * 3)
                empty_length = 30 - bar_length
                sys.stdout.write("\r[%3d]|%s%s|" % (
                    percentage, '█' * bar_length, ' ' * empty_length))
                sys.stdout.flush()
                # 如果概率超过某个临界值，则识别为人声,不断更新human_voice_time 和 not_human_voice_time
                if percentage > sensitive:
                    # 监测到人声一定开始录音
                    begin_flag = True
                    human_voice_time = time.time()
                if percentage < sensitive:
                    not_human_voice_time = time.time()
                # 如果已经开始录音了，并且现在监测到了非人声
                if begin_flag and not_human_voice_time is not None and human_voice_time is not None:
                    # 如果非人声已经持续了2s,就停止监测
                    if not_human_voice_time - human_voice_time >= 2:
                        break

        except KeyboardInterrupt:
            print('Stopping ...')
        finally:
            # 释放资源
            if cobra is not None:
                cobra.delete()

            if wav_file is not None:
                wav_file.close()

            recorder.delete()


if __name__ == '__main__':
    pass
