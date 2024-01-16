import sys
sys.path.append("/home/xm/catkin_ws/src/speech/scripts/Speech_master/nlp")
import whisper as whisper

"""
自动语音识别文件
"""


class ASR:
    """
        用于语音识别
        如果这是个需要加载模型的load_asr_model函数需要被提前启用，以减少加载模型带来的时间损耗
        调用analyse_audio函数进行识别返回文本
    """

    def audio2text(self, audio_path):
        """
        将存在于audio_path的音频文件解析为文本
        """
        pass

    def __init__(self):
        pass

    def load_asr_model(self):
        """
        加载自然语音处理类的模型
        """
        pass


class WhisperAsr(ASR):
    """
    基于OpenAI的ASR
    https://github.com/openai/whisper
    源码已在项目中给出
    可以指定模型大小，但对于简短的语句相差不大
    第一次使用whisper会自动将使用的模型下载到缓存
    """

    def load_asr_model(self, model_name='small'):
        """
        基于whisper的加载模型函数
        模型名称默认为base,tiny效果不大好
        """
        self.model = whisper.load_model(model_name)

    def audio2text(self, audio_path, aim_language = 'en'):
        """
        基于whisper的语音转文本函数
        audio_path应当为语音文件路径
        aim_language可以指定识别成什么语言，默认为英文，可以选择为空自动识别语言
        """
        result = self.model.transcribe(audio_path, language = aim_language)
        return result["text"]

    def __init__(self):
        pass


if __name__ == "__main__":
    # 测试基于whisper的ASR
    asr = WhisperAsr()
    # 测试当前文件夹下的test_whisper.m4a,Elizabeth.m4a
    asr.load_asr_model('base')

    print(asr.audio2text("test_whisper.m4a"))

    print(asr.audio2text("Elizabeth.m4a", ))
