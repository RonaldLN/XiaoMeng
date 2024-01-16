import os
import struct
import wave
from threading import Thread
import pvporcupine
from pvrecorder import PvRecorder


class PorcupineKWS(Thread):
    """
    基于picovoice的豪猪关键词唤醒
    https://picovoice.ai/
    更改自porcupine_demo_mic.py，https://github.com/Picovoice/porcupine/tree/master/demo/python
    所需要的关键词ppn文件需要从官网获得，每个月可以免费定制三个关键词
    需要access_key，我的为"CWwRg5ghpE5IvLl/xqJNHoW6snSeLHvEuS4Us17YphjYW8esgcUFXA==",不需要替换
    """

    def __init__(
            self,
            access_key="CWwRg5ghpE5IvLl/xqJNHoW6snSeLHvEuS4Us17YphjYW8esgcUFXA==",
            library_path=pvporcupine.LIBRARY_PATH,
            model_path=pvporcupine.MODEL_PATH,
            keyword_paths=['./Robot_en_windows_v2_1_0.ppn'],
            sensitive=1,
            output_path=None,
            input_device_index=-1):
        sensitivities = [sensitive] * len(['./Robot_en_windows_v2_1_0.ppn'])
        """
        library_path为porcupine动态库的绝对路径，不要修改
        model_path不用改
        keyword_paths需要给出关键词唤醒ppn文件的路径
        sensitive可以指定灵敏度，从0到1
        input_device_index为麦克风编号，有可能需要改，默认为正在使用的那个
        output_path不用改
        """
        super(PorcupineKWS, self).__init__()
        self._access_key = access_key
        self._library_path = library_path
        self._model_path = model_path
        self._keyword_paths = keyword_paths
        self._sensitivities = sensitivities
        self._input_device_index = input_device_index
        self._output_path = output_path

    def run(self):
        """
         重载自Thread的函数，将会启用新线程，并创建音频输入流监听关键词
         """
        # keywords 可以指定很多个，此版本只支持一个
        keywords = list()
        # 将keyword_paths对应的keyword提取出来
        for x in self._keyword_paths:
            keyword_phrase_part = os.path.basename(x).replace('.ppn', '').split('_')
            if len(keyword_phrase_part) > 6:
                keywords.append(' '.join(keyword_phrase_part[0:-6]))
            else:
                keywords.append(keyword_phrase_part[0])
        # 创建豪猪KWS空引用
        porcupine = None
        # 音频接受器空引用
        recorder = None

        try:
            # 实例化豪猪KWS
            porcupine = pvporcupine.create(
                access_key=self._access_key,
                library_path=self._library_path,
                model_path=self._model_path,
                keyword_paths=self._keyword_paths,
                sensitivities=self._sensitivities)
            # 实例化音频接收器
            recorder = PvRecorder(device_index=self._input_device_index, frame_length=porcupine.frame_length)
            recorder.start()
            print('Listening {')
            for keyword, sensitivity in zip(keywords, self._sensitivities):
                print('  %s (%.2f)' % (keyword, sensitivity))
            print('}')

            while True:
                # pcm为音频帧
                pcm = recorder.read()
                # 分析音频帧，如果识别到关键词，result为关键词在关键词列表中的需要，否则为负数
                result = porcupine.process(pcm)
                if result >= 0:
                    print('检测到关键词')
                    break
        except KeyboardInterrupt:
            print('Stopping ...')
        finally:
            # 回收资源
            if porcupine is not None:
                porcupine.delete()

            if recorder is not None:
                recorder.delete()


if __name__ == '__main__':
    pass
