
from nlp.text_to_speech.audio_player import EspeakAudioPlayer
from nlp.automatic_speech_recognition.asr import WhisperAsr
# from nlp.keyword_spotting import picovoice_kws
from nlp.sound_recorder.webrtc_vad import WebrtcVad
from nlp.keyword_spotting.snowboy_vad import SnowboyKWS


class Listener:
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
class Listener_hand_me_that:
    """
    这是一个单线程版本，不预先加载资源，不使用命令词唤醒
    """

    def listen_once(self):
        # 播放提示音，提示引导者说话
        asr = WhisperAsr()
        asr.load_asr_model()

        audio_player = EspeakAudioPlayer()


        audio_player.play_audio("Hello, I am robot.What is your name?")

        vad = WebrtcVad()

        vad.run()
        print("识别结束")
        text = asr.audio2text("vad.wav")
        audio_player.play_audio("Your name is "+text+"What do you want?")

        vad.run()
        print("识别结束")
        text = asr.audio2text("vad.wav")
        audio_player.play_audio("You want "+text)
        return text



if __name__ == "__main__":
    listener = Listener()
    print("\n", listener.listen_once())
