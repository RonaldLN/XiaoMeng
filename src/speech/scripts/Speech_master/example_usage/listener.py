import sys
sys.path.append("/home/xm/catkin_ws/src/speech/scripts/Speech_master/nlp")
sys.path.append("/home/xm/catkin_ws/src/speech/scripts/Speech_master")
from nlp.text_to_speech.audio_player import EspeakAudioPlayer
from nlp.automatic_speech_recognition.asr import WhisperAsr
# from nlp.keyword_spotting import picovoice_kws
from nlp.sound_recorder.webrtc_vad import WebrtcVad
from nlp.keyword_spotting.snowboy_vad import SnowboyKWS


class Listener_test:
    """
    这是一个单线程版本，不预先加载资源，不使用命令词唤醒
    """

    def listen_once(self):
        # 播放提示音，提示引导者说话
        asr = WhisperAsr()
        asr.load_asr_model()

        audio_player = EspeakAudioPlayer()
        kws = SnowboyKWS()
        kws.run()


        audio_player.play_audio("Please give your instructions.")

        vad = WebrtcVad()

        vad.run()
        print("识别结束")
        text = asr.audio2text("vad.wav")
        audio_player.play_audio("The basketball is in front of you.")
        return text
class Listener:
    def __init__(self):
        self.asr=WhisperAsr()
        self.asr.load_asr_model()
    """
    这是一个单线程版本，不预先加载资源，不使用命令词唤醒
    """

    def listen_object(self):
        # 播放提示音，提示引导者说话

        
        audio_player = EspeakAudioPlayer()
    

        audio_player.play_audio("What do you want?")
        vad = WebrtcVad()
        vad.run()
        text = self.asr.audio2text("vad.wav")
        print("识别结束")
        print(text)
        return text
    def listen_follow(self):

        audio_player = EspeakAudioPlayer()
        audio_player.play_audio("I can follow you.")
        vad = WebrtcVad()
        vad.run()
        text = self.asr.audio2text("vad.wav")
        print("识别结束")
        print(text)
        return text


if __name__ == "__main__":
    listener = Listener()
    print("\n", listener.listen_object())
