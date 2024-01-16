import subprocess

"""
音频播放文件
"""


class AudioPlayer:
    """
    为了节省时间，所有音频文件应当被提前训练
    如果需要即时播放则现场训练
    调用play_audio函数播放音频
    """

    def __init__(self):
        pass

    def play_audio(self, sentence):
        pass


class EspeakAudioPlayer(AudioPlayer):
    """
    采用Espeak的AudioPlayer，无需训练音频，直接利用音素合成
    使用方法：https://espeak.sourceforge.net/commands.html
    使用前应当将Espeak加入环境变量
    调用原理是利用subprocess直接使用命令行
    """

    def play_audio(self, sentence, speed='150', gap='1'):
        """
        参数应当均以字符串的形式给出
        sentence应为UTF-8编码
        speed为每分钟字数，默认为175，下限为80，上限为500
        gap为单词之间插入的停顿，该值为暂停的长度，以10毫秒为单位
        """
        subprocess.call(['espeak', '-s', speed, '-g', gap, sentence])


if __name__ == "__main__":
    eSpeaker_audio_player = EspeakAudioPlayer()
    eSpeaker_audio_player.play_audio('Hello this is a test')
