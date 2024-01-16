from nlp.snowboy_kws import snowboydecoder as snowboydecoder
import sys
import signal

interrupted = False


def signal_handler(signal, frame):
    global interrupted
    interrupted = True


def interrupt_callback():
    global interrupted
    return interrupted


class SnowboyKWS:
    def run(self):
        model = "robot.pmdl"
        # capture SIGINT signal, e.g., Ctrl+C
        signal.signal(signal.SIGINT, signal_handler)

        detector = snowboydecoder.HotwordDetector(model, sensitivity=0.5)
        print('Listening... Press Ctrl+C to exit')

        # main loop
        detector.start(detected_callback=snowboydecoder.play_audio_file,
                       interrupt_check=interrupt_callback,
                       sleep_time=0.03)

        detector.terminate()


if __name__ == '__main__':
    snowboy_vad = SnowboyKWS()
    snowboy_vad.run()
