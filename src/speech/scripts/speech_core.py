#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import sys
sys.path.append("/home/xm/catkin_ws/src/speech/scripts/Speech_master")
sys.path.append("/home/xm/catkin_ws/src/speech/scripts/Speech_master/nlp")
__version__ = 2021.1
__author__ = "Kenny"

import sys
import time
import wave
import chardet
import rospy
import json
from speech.srv import *
from speech_recognition import *#使用该库的Recognizer对象，可以做端点检测
import spacy
#自然语言处理库，用于分析依存关系和词性
import re#正则表达式处理离线识别生成的html格式字符串
from pydub import AudioSegment#音频预处理
from pydub.playback import play#用于播放音频
import fuzzywuzzy.process as fuzzp
from environment_info import * #pycharm中需要把根目录mark为Sources Root（右键）
from Speech_master.example_usage.listener import Listener
from Speech_master.nlp.text_to_speech.audio_player import *
#baiduAI API prams
IS_PY3 = sys.version_info.major == 3
if IS_PY3:
    from urllib.request import urlopen
    from urllib.request import Request
    from urllib.error import URLError
    from urllib.parse import urlencode
    from urllib.parse import quote_plus
else:
    import urllib2
    from urllib import quote_plus
    from urllib2 import urlopen
    from urllib2 import Request
    from urllib2 import URLError
    from urllib import urlencode
API_KEY = 'T4bGqHTwkWhp4OP4ZrzXG4xF'#xpp
SECRET_KEY = 'LppajtBa9IVaCOTGeryxHSZ8qMnOjAUU'
# API_KEY = 'QEZwqA0k32CXoW1vMs37mvmt'#Liuyuhan
# SECRET_KEY = 'FbHWOoRVSZFvgvbHGT4ca3UpptDG097k'
TEXT = "Hello"
# 发音人选择, 基础音库：0为度小美，1为度小宇，3为度逍遥，4为度丫丫，
# 精品音库：5为度小娇，103为度米朵，106为度博文，110为度小童，111为度小萌，默认为度小美
PER = 1
# 语速，取值0-15，默认为5中语速
SPD = 4
# 音调，取值0-15，默认为5中语调
PIT = 7
# 音量，取值0-9，默认为5中音量
VOL = 9
# 下载的文件格式, 3：mp3(default) 4： pcm-16k 5： pcm-8k 6. wav
AUE = 6
FORMATS = {3: "mp3", 4: "pcm", 5: "pcm", 6: "wav"}
FORMAT = FORMATS[AUE]
CUID = "123456PYTHON"

TTS_URL = 'http://tsn.baidu.com/text2audio' 
TOKEN_URL = 'http://openapi.baidu.com/oauth/2.0/token'
SCOPE = 'audio_tts_post'  # 有此scope表示有tts能力，没有请在网页里勾选

class DemoError(Exception):
    pass

# 连接网站
def fetch_token():
    params = {'grant_type': 'client_credentials',
              'client_id': API_KEY,
              'client_secret': SECRET_KEY}
    # 是指针对网页url中的中文字符的一种编码转化方式
    post_data = urlencode(params)
    if (IS_PY3):
        post_data = post_data.encode('utf-8')
    req = Request(TOKEN_URL, post_data)
    try:
        f = urlopen(req, timeout=5)
        result_str = f.read()
    except URLError as err:
        print('token http response http code : ' + str(err.code))
        result_str = err.read()
    if (IS_PY3):
        result_str = result_str.decode()


    result = json.loads(result_str)

    if ('access_token' in result.keys() and 'scope' in result.keys()):
        if not SCOPE in result['scope'].split(' '):
            raise DemoError('scope is not correct')

        return result['access_token']
    else:
        raise DemoError('MAYBE API_KEY or SECRET_KEY not correct: access_token or scope not found in token response')

class XM(Recognizer):
    def __init__(self):
        Recognizer.__init__(self)
        self.res = ""  # 语音识别的文本结果
        self.ans = ""  # 需语音合成的文本
        self.cmd = []  # 用于此处分析出完整的cmd信息
        # information for smach
        self.act = []  # 动作序列
        self.obj = []  # 对象序列
        self.rem = []  # 备注（暂未用到）
        self.num = 0  # 任务个数
        self.nlp = spacy.load('en_core_web_sm')  # spacy分析对象内置
        self.gesture = ""
        # receptionist
        self.name = ""
        self.drink = ""
        self.listener=Listener()
    def clear(self):
        self.res = ""  # 语音识别的文本结果
        self.ans = ""  # 需语音合成的文本
        self.cmd = []  # 用于此处分析出完整的cmd信息
        # information for smach
        self.act = []  # 动作序列
        self.obj = []  # 对象序列
        self.rem = []  # 备注（暂未用到）
        self.num = 0  # 任务个数
        self.gesture = ""
        # receptionist
        self.name = ""
        self.drink = ""
    def listen_and_record(self):#麦克风获取原始音频信息并保存到本地wav文件

        m = Microphone(sample_rate=16000, chunk_size=1024)#获取麦克风

        with m as s:
            print("=====Please wait a minute...")
            # self.adjust_for_ambient_noise(m,2)#检测噪声
            record=AudioSegment.from_wav("/home/xm/catkin_ws/src/speech/tts_record/cue_tone.wav")
            play(record)
            print("+++++Now start to listen...")
            # time.sleep(2)
            audio = self.listen(m, None, 5, None)#录制音频
            print("=====Got it!Now start to recognize...")
        raw=audio.get_raw_data(None,None)#获取原始音频数据
        wav_file="/home/xm/catkin_ws/src/speech/raw_record/recorded_voice.wav"#原始音频存储路径
        #如果有必要可以微调音频格式再存储
        wav_writer = wave.open(wav_file, "wb")
        wav_writer.setframerate(audio.sample_rate)
        wav_writer.setsampwidth(audio.sample_width)
        wav_writer.setnchannels(1)
        wav_writer.writeframes(raw)#对原始音频数据做微调
        wav_writer.close()
        print("listen already")
    def ASR_offline(self,bnf):#离线语音识别，可指定语法文件，识别效果不好
        self.listen_and_record()
        rospy.wait_for_service('sr_offline_file')#等待服务
        sr_online = rospy.ServiceProxy("sr_offline_file", speech)#生成服务对象
        if bnf==" ": resp = sr_online("START!")#服务传入request参数
        elif bnf=="gpsr":resp = sr_online(bnf)
        elif bnf=="question":resp = sr_online(bnf)
        html=str(resp.output)#获取服务的response
        L=re.findall(r"\d+\">(.+?)</",html,re.S)#对html字符串做正则化提取，注意要非贪婪匹配，re.S防止遇到换行符停止匹配
        sent=' '.join(L)#匹配到的字符串以空格连接
        return sent
    def ASR_online(self):#在线语音识别，效果好，对网络要求高
        self.listen_and_record()
        rospy.wait_for_service('sr_online_file')
        sr_online = rospy.ServiceProxy("sr_online_file", speech)
        req = speechRequest()
        req.source = "START!"
        resp = sr_online.call(req)
        print("在线响应结果:%s"%(resp.output))
        return str(resp.output).lower()
    def TTS(self,text):#调用语音合成服务，传入需合成的文本
        # rospy.wait_for_service('tts')
        # sr_online = rospy.ServiceProxy("tts", speech)
        # resp = sr_online(text)
        # return str(resp.output)#传回生成文件的地址

        TEXT=text
        token = fetch_token()
        tex = quote_plus(TEXT)  # 此处TEXT需要两次urlencode

        params = {'tok': token, 'tex': tex, 'per': PER, 'spd': SPD, 'pit': PIT, 'vol': VOL, 'aue': AUE, 'cuid': CUID,
                          'lan': 'zh', 'ctp': 1}  # lan ctp 固定参数
            # "tex":  # 要进行语音合成的内容
            # "tok":  # 个人的鉴权认证Acess Token
            # "cuid":# 随便一个值就好了，官网推荐是个人电脑的MAC地址
            # "ctp":1,   # 客户端类型，web端固定值1
            # "lan":"zh",   # 中文语言
            # "spd":5,   # 语速
            # "pit":5,   # 语调
            # "vol":5,   # 音量
            # "per":4,   # 男女声，4是度丫丫
            # "aue":3,   # 音频格式，3是mp3
        data = urlencode(params)


        req = Request(TTS_URL, data.encode('utf-8'))
        has_error = False
        try:
            f = urlopen(req)
            result_str = f.read()

            headers = dict((name.lower(), value) for name, value in f.headers.items())

            has_error = ('content-type' not in headers.keys() or headers['content-type'].find('audio/') < 0)
        except  URLError as err:
            print('asr http response http code : ' + str(err.code))
            result_str = err.read()
            has_error = True

        save_file = "error.txt" if has_error else '/home/xm/catkin_ws/src/speech/tts_record/baidu_tts_result.' + FORMAT
        # path='_'.join(text.split())+'.'
        # save_file="error.txt" if has_error else '/home/kenny/Desktop/ws_XfSpeech/tts_record/gpsr_questions/'+ path + FORMAT
        # print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$4")
        # print(save_file)
        with open(save_file, 'wb') as of:
            of.write(result_str)

        if has_error:
            if (IS_PY3):
                result_str = str(result_str, 'utf-8')
            #print("tts api  error:" + result_str)


        return save_file
        #print("result saved as :" + save_file)
    #用于调试时分析spacy处理结果（开黑盒）
    def spacy_analyze(self,sent):
        doc=self.nlp(sent.lower().encode("utf-8").decode("utf-8"))


        L=[]
        for w in doc:
            L.append((w,w.tag_,w.dep_,w.head))#词本身，词性标注，词的依存关系，依存关系中上一个词
        return L
    #应用spacy分析结果提取cmd文本关键信息传给状态机（黑盒）
    def NLP(self, sent):
        try:
            sent = sent.lower()
            # print(sent)
            # 回答队名、时间、日期等问题
            sent = sent.replace('.', ' ')
            sent = sent.replace('?', ' ')
            sent = sent.replace(',', ' ')


            for n in gestures:
                if "person " + n in sent:
                    sent = sent.replace("person " + n, gestures[n] + "_person", 1)
                    break
            for n in poses:
                if "person " + n in sent:
                    sent = sent.replace("person " + n, poses[n] + "_person", 1)
                    break
            if "all the" in sent: sent = sent.replace("all the", "all", 1)
            if "soup" in sent: sent = sent.replace("soup", "soap", 1)

            # if "male person" in sent:sent=sent.replace("male person","male_person",1)
            # if "female person" in sent:sent=sent.replace("female person","female_person",1)
            sent=sent.replace(" code"," coke",1)
            sent = sent.replace(" cold", " coke", 1)
            sent = sent.replace(" cook", " coke", 1)
            sent=sent.replace("asked ","escort ",1)
            sent=sent.replace(" death"," desk",1)
            for n in ["alex","charlie","elizabeth","francis","jennifer","linda","mary","patricia","robin","skyler","james","john","michael","robert","william"]:
                if n in sent:
                    sent=sent.replace(n,"the person",1)
            doc = self.nlp(sent.strip().lower().encode("utf-8").decode("utf-8"))  # 生成spacy分析结果
            # for n in self.spacy_analyze(sent):
            #     print(n)
            # #doc = self.nlp(sent.decode("gbk"))

            act_index = 0  # 用于记录动词（中心词）的索引
            name_list = []  # 记录分析出的所有名词（房间名，大小物体名，person、question额外名词）
            prep_list = []  # 记录介词，用于区分take等有歧义的动词
            compound_flag = ""  # 复合词标记
            amod_flag = ""  # 形容词修饰物体标记
            pronoun_obj = ""  # 代词指代对象记忆化
            pronoun_per = ""  # 代词指代人名记忆化
            pronoun_beacon = ""  # guide back的beacon记忆化
            beacon_flag_you = ""  # guide里面一个特殊句式
            # oprop_flag=""#最高级合成词标记
            introduced_flag = ""  # introduce里有双人名，标记前一个人名
            '''
            if "stop" in sent:
                self.act.append("stop")
                return
            '''
            if "stop" in sent or "store" in sent or "shop" in sent or "arrived" in sent or "arrive" in sent:
                self.num += 1
                self.obj.append("")
                self.act.append("stop")
                return
            # print(sent)
            # 以每个root中心词为处理单元（遇到一个中心词，处理上一个中心词对应的句子），因此最后一句需要单独处理，令i超出doc范围
            for i in range(len(doc) + 1):
                if i == len(doc) or (doc[i].dep_ == "ROOT" or doc[i].dep_ == "conj" or doc[i].dep_ == "advcl") and (
                        doc[i].tag_ == "VB" or doc[i].tag_ == "VBD" or doc[i].tag_ == "NNP"):  # 动词（root中心词或并列连接词）
                    act = ""
                    # 单句处理
                    cmd_dict = {"act": "", "obj": "", "per": "", "room": [], "beacon": [], "place": [],
                                "ans": ""}  # 每个cmd对应的传出格式
                    if name_list != [] or self.ans != "":
                        if doc[act_index].text in people and actions[doc[act_index - 1].text] in actions:
                            name_list.append(doc[act_index].text)
                            act_index -= 1

                        print(name_list)
                        # 明晰可能产生歧义的动词
                        if doc[act_index].text == "bring":
                            if doc[act_index + 1].text in objects:
                                act = "bring_o"
                                act = actions[act]
                            elif doc[act_index + 1].text in people:
                                act = "bring_p"
                                act = actions[act]
                        elif doc[act_index].text == "take":
                            if doc[act_index + 1].text in objects:
                                act = "take_o"
                                act = actions[act]
                            elif doc[act_index + 1].text in people:
                                act = "take_p"
                                act = actions[act]
                        elif doc[act_index].text == "look":
                            if doc[act_index + 1].text == "for":
                                act = "look_for"
                                act = actions[act]
                        else:
                            act = doc[act_index].text
                        if act in actions:
                            act = actions[act]  # 提取动词对应的类别
                        else:
                            act = fuzzp.extractOne(act, list(actions.keys()))[0]
                            # 明晰可能产生歧义的动词
                            if doc[act_index].text == "bring":
                                if doc[act_index + 1].text in objects:
                                    act = "bring_o"
                                    act = actions[act]
                                elif doc[act_index + 1].text in people:
                                    act = "bring_p"
                                    act = actions[act]
                            elif doc[act_index].text == "take":
                                if doc[act_index + 1].text in objects:
                                    act = "take_o"
                                    act = actions[act]
                                elif doc[act_index + 1].text in people:
                                    act = "take_p"
                                    act = actions[act]
                            elif doc[act_index].text == "look":
                                if doc[act_index + 1].text == "for":
                                    act = "look_for"
                                    act = actions[act]
                            else:
                                act = doc[act_index].text
                            act = actions[act]
                        cmd_dict["act"] = act

                        for w in name_list:

                            if act == "guide" and " back" in sent:
                                if pronoun_beacon != "": cmd_dict["beacon"].append(pronoun_beacon)
                            if w in rooms:
                                cmd_dict["room"].append(w)
                            elif w in beacons and not (len(cmd_dict["beacon"]) > 0 and act != "guide"):
                                if act == "find" and " you" in sent:
                                    beacon_flag_you = w
                                    break
                                if " back" in sent:
                                    if act == "meet": pronoun_beacon = w
                                    # if act=="guide":w=pronoun_beacon
                                cmd_dict["beacon"].append(w)
                            elif w in placements:
                                cmd_dict["place"].append(w)
                            elif w in people:
                                if cmd_dict["per"] != "": introduced_flag = cmd_dict["per"]
                                if (w == "him" or w == "her") and pronoun_per != "": w = pronoun_per
                                if w in people: cmd_dict["per"] = w
                                pronoun_per = w
                            elif w in list(objects.keys()) + list(categories.keys()):
                                if w == "it" and pronoun_obj != "": w = pronoun_obj
                                if w in list(objects.keys()) + list(categories.keys()): cmd_dict["obj"] = w
                                pronoun_obj = w
                            elif act != "tell":
                                w = fuzzp.extractOne(w, list(beacons.keys()) + list(placements.keys()) + list(
                                    objects.keys()) + people + rooms)[0]
                                if w in rooms:
                                    cmd_dict["room"].append(w)
                                elif w in beacons and not (len(cmd_dict["beacon"]) > 0 and act != "guide"):
                                    if act == "find" and " you" in sent:
                                        beacon_flag_you = w
                                        break
                                    if " back" in sent:
                                        if act == "meet": pronoun_beacon = w
                                        # if act=="guide":w=pronoun_beacon
                                    cmd_dict["beacon"].append(w)
                                elif w in placements:
                                    cmd_dict["place"].append(w)
                                elif w in people:
                                    if cmd_dict["per"] != "": introduced_flag = cmd_dict["per"]
                                    if (w == "him" or w == "her") and pronoun_per != "": w = pronoun_per
                                    if w in people: cmd_dict["per"] = w
                                    pronoun_per = w
                                elif w in list(objects.keys()) + list(categories.keys()):
                                    if w == "it" and pronoun_obj != "": w = pronoun_obj
                                    if w in list(objects.keys()) + list(categories.keys()): cmd_dict["obj"] = w
                                    pronoun_obj = w

                        if beacon_flag_you == "": self.cmd.append(cmd_dict)  # 添加到cmd传出列表
                        # name_list.clear()#清空
                        del name_list[:]
                        prep_list = []  # 记录介词，用于区分take等有歧义的动词
                        compound_flag = ""  # 复合词标记
                        # amod_flag = ""  # 形容词修饰物体标记
                        # pronoun_obj = ""  # 代词指代对象记忆化
                        # pronoun_per = ""  # 代词指代人名记忆化
                    act_index = i  # 更新中心词索引

                elif doc[i].dep_ == "prep":
                    prep_list.append(doc[i].text)
                elif doc[i].dep_ == "amod" and doc[i].text != "many" or doc[i].text == "all" and doc[i].dep_ == "predet":
                    amod_flag = doc[i].head.text
                    name_list.append(doc[i].text)
                elif doc[i].dep_ == "compound" and doc[i].tag_ != "VB":
                    compound_flag = doc[i].head.text
                    name_list.append(doc[i].text)
                elif doc[i].text in rooms+list(beacons.keys())+list(placements.keys())+list(objects.keys())+list(categories.keys())+people or doc[i].dep_ == "dobj" or doc[i].dep_ == "pobj" or doc[i].dep_ == "advcl" or doc[
                    i].dep_ == "npadvmod" or doc[i].dep_ == "attr" or (
                        doc[i].dep_ == "dative" and doc[i - 1].tag_ == "VB") or (
                        doc[i].dep_ == "acl" and doc[i - 1].text == "to"):
                    if amod_flag != "" and doc[i].text == amod_flag:
                        name_list[-1] += '_' + amod_flag  # 默认用下划线链接复合词
                        amod_flag = ""
                    elif compound_flag != "" and doc[i].text == compound_flag:
                        name_list[-1] += '_' + compound_flag  # 默认用下划线链接复合词
                        compound_flag = ""
                    elif doc[i].text == "bag" and doc[i - 1].text == "my":
                        name_list.append("my_bag")
                    else:
                        name_list.append(doc[i].text)
            # print(self.cmd)
            # 构造需传递给smach的信息
            print(self.cmd)
            get_flag = 0
            find_flag = 0
            for base in self.cmd:
                action, person, object, room, beacon, placement, answer = base["act"], base["per"], base["obj"], base[
                    "room"], base["beacon"], base["place"], base["ans"]
                if action == "get" and get_flag == 1: action = "deliver"
                if action == "answer":
                    if person == "" and room == []:
                        self.act.append(action)
                        self.obj.append(object)
                        self.num += 1
                    else:
                        if room != []:
                            self.act.append("go")
                            self.obj.append(room[0])
                            self.num += 1
                        if person != "":
                            self.act.append("find")
                            self.obj.append(person)
                            self.num += 1
                        self.act.append(action)
                        self.obj.append(object)
                        self.num += 1
                elif action == "tell":
                    if room != []:
                        self.act.append("go")
                        self.obj.append(room[0])
                        self.num += 1
                    elif beacon != []:
                        self.act.append("go")
                        self.obj.append(beacons[beacon[0]] + "_" + beacon[0])
                        self.num += 1
                    elif placement != []:
                        self.act.append("go")
                        self.obj.append(placements[placement[0]] + "_" + placement[0])
                        self.num += 1
                    sentence = ""

                    if "of the person" in sent:
                        self.act.append("find")
                        self.obj.append("person")
                        self.num += 1
                        if "name" in sent:
                            sentence = "name"
                            self.gesture = "name"
                        if "gender" in sent: sentence = "gender"
                        if "pose" in sent: sentence = "pose"
                        # self.act.append("recognize")  # new?describe?introduce?
                        # self.obj.append(sentence)
                        # self.num += 1
                        if " me" in sent and person != "person": person = "me"
                    elif "what's" in sent or "which" in sent:
                        if "three" in sent: sentence = "three"
                        for n in oprop:
                            if n in sent:
                                if sentence != "":
                                    sentence += "_" + n
                                else:
                                    sentence = n
                        if "object" in sent:
                            sentence += "_object"
                        elif "objects" in sent:
                            sentence += "_objects"
                        else:
                            for n in categories:
                                if n in sent: sentence += "_" + n
                        self.act.append("find")  # new?
                        self.obj.append(sentence)
                        self.num += 1
                    elif "how many people" in sent:
                        d = {"men": "man", "women": "woman", "boys": "boy", "girls": "girl", "male": "male_person",
                             "female": "female_person", "sitting": "sitting_person", "standing": "standing_person",
                             "lying down": "lying_down_person"}
                        for n in d:
                            if n in sent:
                                sentence = d[n]
                        self.act.append("count")  # new?
                        self.obj.append(sentence)
                        self.num += 1
                        sentence = "num"
                    elif "how many" in sent and object != "":
                        self.act.append("count")
                        self.obj.append(object)
                        self.num += 1
                        sentence = "num"
                    else:
                        t = time.localtime(time.time())
                        sentence = "ans"
                        if "something about yourself" in sent: self.ans = "my name is 晓萌,I come from 西安,China"
                        if "time" in sent: self.ans = "It is " + str(t.tm_hour) + ' ' + str(t.tm_min) + " now"
                        if "what day is" in sent:
                            if "today" in sent:
                                self.ans = "the 21st May"
                            elif "tomorrow" in sent:
                                self.ans = "the 22nd May"
                        if " week" in sent: self.ans="Friday"
                        if " month" in sent: self.ans="May"
                        if "a joke" in sent: self.ans = "the boy says can I buy you a drink and the girl says I'd rather have the money"
                        if "your team" in sent or "teens" in sent:
                            if "name" in sent: self.ans = "晓萌 team"
                            if "country" in sent: self.ans = "China"
                            if "affiliation" in sent: self.ans = "northwestern polytechnical university"
                    if person == "me":
                        self.act.append("go")
                        self.obj.append("information_pos")
                        self.num += 1
                    if person != "" and person != "me":
                        self.act.append("find")  # find me怎么解决？
                        self.obj.append(person)
                        self.num += 1
                    self.act.append(action)
                    self.obj.append(sentence)
                    self.num += 1
                elif action == "find":  # go room,find 小物体 的可能性？要不要默认位置？
                    if room != []:
                        self.act.append("go")
                        self.obj.append(room[0])
                        self.num += 1
                    elif beacon != []:
                        self.act.append("go")
                        m = beacons[beacon[0]]
                        if m == "door":
                            self.obj.append(beacon[0])
                        else:
                            self.obj.append(m + "_" + beacon[0])
                        self.num += 1
                    if person != "":
                        self.act.append(action)
                        self.obj.append(person)
                        self.num += 1
                    elif object != "":
                        find_flag = 1
                        if beacon == [] and placement == []:
                            pass  # 是否要在find物品是添加默认位置？
                        self.act.append(action)
                        self.obj.append(object)
                        self.num += 1
                elif action == "go":  # gobeacon和goplacement已添加默认房间
                    if room != []:
                        self.act.append(action)
                        self.obj.append(room[0])
                        self.num += 1
                    elif beacon != []:
                        self.act.append(action)
                        self.obj.append(beacons[beacon[0]] + "_" + beacon[0])
                        self.num += 1
                    elif placement != []:
                        self.act.append(action)
                        self.obj.append(placements[placement[0]] + "_" + placement[0])
                        self.num += 1
                elif action == "meet":
                    if beacon != []:
                        self.act.append("go")
                        self.obj.append(beacons[beacon[0]] + "_" + beacon[0])
                        self.num += 1
                    if person != "":
                        self.act.append("find")
                        self.obj.append(person)
                        self.num += 1
                elif action == "guide":
                    if len(beacon) == 2:
                        self.act.append("go")
                        self.obj.append(beacons[beacon[0]] + "_" + beacon[0])
                        self.num += 1
                        if person != "":
                            self.act.append("find")
                            self.obj.append(person)
                            self.num += 1
                        self.act.append(action)
                        self.obj.append(beacons[beacon[1]] + "_" + beacon[1])
                        self.num += 1
                    elif len(beacon) == 1:
                        if beacon_flag_you != "":
                            self.act.append("go")
                            self.obj.append(beacons[beacon_flag_you] + "_" + beacon_flag_you)
                            self.num += 1
                            self.act.append("find")
                            self.obj.append(person)
                            self.num += 1
                        # if person!="" and person not in ["her","him","it"]:
                        #     self.act.append("find")
                        #     self.obj.append(person)
                        #     self.num += 1
                        self.act.append(action)
                        if beacons[beacon[0]] == "taxi":
                            self.obj.append(beacon[0])
                        else:
                            self.obj.append(beacons[beacon[0]] + "_" + beacon[0])
                        self.num += 1
                elif action == "follow":
                    if beacon != [] and room != []:
                        self.act.append("go")
                        self.obj.append(beacons[beacon[0]] + "_" + beacon[0])
                        self.num += 1
                        if person != "":
                            self.act.append("find")
                            self.obj.append(person)
                            self.num += 1
                        self.act.append(action)
                        self.obj.append(person)
                        self.num += 1  # follow sb. to room,到底要不要加room？
                    else:
                        self.act.append(action)
                        self.obj.append(person)
                        self.num += 1  # follow sb. to room,到底要不要加room？
                elif action == "get":
                    get_flag = 1
                    if placement == [] and beacon != [] and beacon[0] in placements:
                        placement = beacon[:]
                    if objects[object] == "luggage":  # luggage?
                        self.act.append(action)
                        self.obj.append(object)
                        self.num += 1
                        self.act.append("go")
                        self.obj.append(beacon[0])
                        self.num += 1
                        self.act.append("deliver")
                        self.obj.append(beacon[0])
                        self.num += 1
                    elif room != [] and placement != []:
                        self.act.append("go")
                        self.obj.append(room[0])
                        self.num += 1
                        self.act.append(action)
                        self.obj.append(object)
                        self.num += 1
                        self.act.append("go")
                        self.obj.append(placements[placement[0]] + "_" + placement[0])
                        self.num += 1
                        self.act.append("deliver")
                        self.obj.append(placements[placement[0]] + "_" + placement[0])
                        self.num += 1
                    elif placement != []:
                        self.act.append("go")
                        self.obj.append(placements[placement[0]] + "_" + placement[0])
                        self.num += 1
                        self.act.append(action)
                        self.obj.append(object)
                        self.num += 1
                    else:  # 加物体默认位置
                        cate = objects[object]
                        loc = categories[cate]
                        r = ""
                        if loc in beacons:
                            r = beacons[loc]
                        elif loc in placements:
                            r = placements[loc]
                        if find_flag == 0:
                            self.act.append("go")
                            self.obj.append(r + "_" + loc)
                            self.num += 1
                        self.act.append(action)
                        self.obj.append(object)
                        self.num += 1
                        if person != "":
                            if person == "me":
                                self.act.append("go")
                                self.obj.append("information_pos")
                                self.num += 1
                            elif room != []:
                                self.act.append("go")
                                self.obj.append(room[0])
                                self.num += 1
                            if person != "me":
                                self.act.append("find")
                                self.obj.append(person)
                                self.num += 1
                            self.act.append("deliver")
                            self.obj.append(person)
                            self.num += 1
                elif action == "deliver":
                    if placement == [] and beacon != [] and beacon[0] in placements:
                        placement = beacon[:]
                    if "from" in sent and "me" in sent and placement != []:
                        self.act.append("go")
                        self.obj.append(placements[placement[0]] + "_" + placement[0])
                        self.num += 1
                        self.act.append("get")
                        self.obj.append(object)
                        self.num += 1
                        if person == "me":
                            self.act.append("go")
                            self.obj.append("information_pos")
                            self.num += 1
                        if person != "me":
                            self.act.append("find")
                            self.obj.append(person)
                            self.num += 1
                        self.act.append(action)
                        self.obj.append(person)
                        self.num += 1
                    # elif objects[object]=="luggage":#luggage?
                    #     self.act.append("get")
                    #     self.obj.append(object)
                    #     self.num += 1
                    #     self.act.append("go")
                    #     self.obj.append(beacon[0])
                    #     self.num += 1
                    #     self.act.append(action)
                    #     self.obj.append(beacon[0])
                    #     self.num += 1
                    else:
                        if len(self.act) == 0:
                            cate = object
                            if object in objects: cate = objects[object]
                            loc = categories[cate]
                            r = ""
                            if loc in beacons:
                                r = beacons[loc]
                            elif loc in placements:
                                r = placements[loc]
                            self.act.append("go")
                            self.obj.append(r + "_" + loc)
                            self.num += 1
                            self.act.append("get")
                            self.obj.append(object)
                            self.num += 1
                        if person != "":
                            if room != []:
                                self.act.append("go")
                                self.obj.append(room[0])
                                self.num += 1
                            elif beacon != []:
                                self.act.append("go")
                                self.obj.append(beacons[beacon[0]] + "_" + beacon[0])
                                self.num += 1
                            if person == "me":
                                self.act.append("go")
                                self.obj.append("information_pos")
                                self.num += 1
                            if person != "me":
                                self.act.append("find")
                                self.obj.append(person)
                                self.num += 1
                            self.act.append(action)
                            self.obj.append(person)
                            self.num += 1
                        elif placement != []:
                            self.act.append("go")
                            self.obj.append(placements[placement[0]] + "_" + placement[0])
                            self.num += 1
                            self.act.append(action)
                            self.obj.append(placements[placement[0]] + "_" + placement[0])
                            self.num += 1
                elif action == "ask_leave":
                    if "leave" in sent and person != "":
                        self.act.append(action)
                        self.obj.append(person)
                        self.num += 1
                elif action == "introduce":
                    if room != []:
                        self.act.append("go")
                        self.obj.append(room[0])
                        self.num += 1
                    elif beacon != []:
                        self.act.append("go")
                        self.obj.append(beacons[beacon[0]] + "_" + beacon[0])
                        self.num += 1
                    if person != "" and introduced_flag != "":
                        self.act.append("attend")  # ??
                        self.obj.append(person)
                        self.num += 1
                        self.act.append(action)
                        self.obj.append(introduced_flag)
                        self.num += 1
                elif action == "release":
                    self.act.append("release")
                    self.num+=1
            for i, base in enumerate(self.obj):
                if base in people and base != "person":
                    if base in ["man", "woman", "boy", "girl", "male_person", "female_person"]:
                        self.obj[i] = "person"
                        self.gesture = base
                    elif "person" in base:
                        l = base.split('_')
                        self.obj[i] = "person"
                        self.gesture = "_".join(l[:-1])
                # if base["ans"]!="":
                #     self.act.append(base["act"])
                #     self.ans=base["ans"]
                #     self.obj.append(base["obj"])
                #     self.num+=1
                #     continue
                # if base["room"]!="":
                #     self.act.append("go")
                #     self.obj.append(base["room"])
                #     self.num+=1
                # if base["loc"]!="":
                #     if base["loc"] in locations:
                #         self.act.append("go")
                #         self.obj.append(base["loc"])
                #     elif base["loc"] in people:
                #         self.act.append("find")
                #         self.obj.append(base["loc"])
                #     self.num += 1
                # if base["obj"] != "":
                #     self.act.append(base["act"])
                #     self.obj.append(base["obj"])
                #     self.num += 1
        except:
            print("NLP Error!")
    def NLPnew(self,sent):
        # shopping 使用
        sent=sent.lower()
        sent = sent.replace("chips", " chip ", 1)
        sent = sent.replace("cheap", " chip ", 1)
        sent = sent.replace("caller", " cola ", 1)

        # sent = sent.replace("some", "  ", 1)


        sent = sent.replace("trip", " chip ", 1)

        sent = sent.replace("orange juice", " orange_juice ", 1)
        sent = sent.replace("surprise", " sprite ", 1)
        sent = sent.replace("price", " sprite ", 1)
        sent = sent.replace("ride", " sprite ", 1)
        sent = sent.replace("so", " dishsoap ", 1)
        sent = sent.replace("cooking", " cookie ", 1)

        sent = sent.replace("soap", " dishsoap ", 1)




        sent = sent.replace("prize", " sprite ", 1)
        sent = sent.replace("spread", " sprite ", 1)
        sent = sent.replace("spite", " sprite ", 1)
        # sent = sent.replace("this", " lays ", 1)
        sent = sent.replace("raise", " lays ", 1)
        sent = sent.replace("lids", " lays ", 1)
        sent = sent.replace("needs", " lays ", 1)
        sent = sent.replace("name", " lays ", 1)
        sent = sent.replace("names", " lays ", 1)
        sent = sent.replace("knees", " lays ", 1)
        sent = sent.replace("lens", " lays ", 1)
        sent = sent.replace("maze", " lays ", 1)
        sent = sent.replace("hand wash", " handwash ", 1)
        sent = sent.replace("dish soap", " dishsoap ", 1)
        sent = sent.replace("dis soap", " dishsoap ", 1)
        sent = sent.replace("dish shop", " dishsoap ", 1)
        sent = sent.replace("spaceship", " dishsoap ", 1)
        sent = sent.replace("color", " cola ", 1)
        sent = sent.replace("potato", " potatowish ", 1)
        sent = sent.replace("real", " oreo ", 1)
        sent = sent.replace("wish"," potatowish ",1)
        sent=sent.replace("lace"," lays ",1)
        sent=sent.replace("race"," lays ",1)
        sent=sent.replace("raze"," lays ",1)
        try:
            sent = sent.lower()
            sent = sent.replace('.', '')
            sent = sent.replace('?', '')
            doc = self.nlp(sent.strip().lower().encode("utf-8").decode("utf-8"))  # 生成spacy分析结果
            for unit in doc:
                word=unit.text
                if word in actions:
                    self.num+=1
                    self.act.append(actions_new[word])
                elif word in objects_new:
                    if word == "orange_juice":
                        word = "orange juice"
                    self.obj.append(word)
                elif word in rooms_new:
                    self.obj.append(word)
                # elif word in gesture_new:
                #     self.gesture=word
            if self.num==0 and ("is" in sent or "are" in sent):
                self.num+=1
                self.act.append("remember")
                self.gesture = 'left'
        except:
            pass
    def Questions(self,sent):
        ques = fuzzp.extractOne(sent, list(questions.keys()))[0]
        print(fuzzp.extractOne(sent, list(questions.keys())))
        ans=questions[ques]
        print("Q:"+ques)
        print("A:"+ans)
        path='_'.join(ans.split())+'.'
        try:
            save_file= '/home/xm/catkin_ws/src/speech/tts_record/gpsr_questions/'+ path + 'wav'
            record = AudioSegment.from_wav(save_file)
            play(record)
        except:
            record = AudioSegment.from_wav(self.TTS(ans))
            play(record)
            print("No existing recorded wav file!")
    def Receptionist(self,num,sent):
        sent=sent.replace("skylar","skyler",1)
        sent=sent.replace("stella","skyler",1)
        sent=sent.replace("cook","coke",1)
        doc = self.nlp(sent.strip().lower().encode("utf-8").decode("utf-8"))  # 生成spacy分析结果
        if num == 1:
            for i in range(len(doc)):
                #print(doc[i].text)
                if doc[i].dep_ == "dobj" or doc[i].dep_ == "pobj" or doc[i].dep_ == "attr" or doc[i].dep_ == "acomp" or doc[i].dep_ == "ROOT":
                    if doc[i].tag_=="VB" or doc[i].tag_ == "VBZ" or doc[i].tag_=="VBP": continue
                    w = doc[i].text
                    if w in people: self.name = w
            #print(self.name)
            if self.name == "":
                for i in range(len(doc)):
                    if doc[i].dep_ == "dobj" or doc[i].dep_ == "pobj" or doc[i].dep_ == "acomp" or doc[i].dep_ == "attr" or doc[i].dep_ == "ROOT":
                        if doc[i].tag_ == "VB" or doc[i].tag_ == "VBZ" or doc[i].tag_ == "VBP": continue
                        w = fuzzp.extractOne(doc[i].text, people)[0]
                        if w in people: self.name = w
            #print(self.name)
        elif num==2:
            for i in range(len(doc)):
                if doc[i].dep_=="dobj" or doc[i].dep_=="pobj" or doc[i].dep_=="compound" or doc[i].dep_=="amod" or doc[i].dep_ == "acomp" or doc[i].dep_=="ROOT":
                    if doc[i].tag_ == "VB" or doc[i].tag_ == "VBP" or doc[i].tag_ == "VBZ" or doc[i].text=="favorite": continue
                    #print(doc[i].text)
                    w=doc[i].text

                    if self.drink=="": self.drink=w
                    else: self.drink+="_"+w
            #print(self.drink)
            pp=[n for n in list(objects.keys()) if objects[n]=="drinks"]
            #print(pp)
            if self.drink not in pp:
                self.drink=fuzzp.extractOne(self.drink, pp)[0]
            #print(self.drink)
xm_globle=XM()
def call_back(req):

    xm_res=speech_to_smachResponse()
    localtime = time.localtime(time.time())
    #print(localtime.tm_year,localtime.tm_mon,localtime.tm_mday,localtime.tm_hour,localtime.tm_min,localtime.tm_sec,localtime.tm_wday)
    if req.command==1:#gpsr recognition3
        xm=XM()
        #sent="navigate to the kitchen, count the male people and report it to skyler which is in the diningroom"
        # sent = "navigate to the livingroom, look for a person and tell the sex of that person"
        #sent="count the male people"
        #sent="get the green tea from the living room"
        #sent="reach the diningroom, find a person and tell which gesture is performing that person"
        # sent="count the rising left arm people in the dining room"
        # sent="Tell me how many Apple are in the kitchen counter"
        # sent="mhm, my name is alex."
        # sent="Tell me how many apple are in the kitchen counter"
        # sent="go to sofa, meet james lead him"
        #sent=xm.ASR_offline("gpsr")
        # sent=xm.ASR_offline(" ")
        listener=Listener()
        sent=listener.listen_follow().lower()

        sent=sent.replace("robert, please","",1)
        sent=sent.strip()
        print("Recognized result: "+sent)

        n = xm.spacy_analyze(sent)

        #for nn in n:
        #    print(nn)
        print(" NLP start!")
        xm.NLP(sent)
        print("NLP over!")

        print(xm.num)
        print(xm.act)
        print(xm.obj)
        print(xm.ans)
        print(xm.gesture)

        xm_res.num=xm.num
        xm_res.action=xm.act
        xm_res.object=xm.obj
        xm_res.answer=xm.ans
        xm_res.gesture=xm.gesture
    elif req.command==2:#tts
 
        text=str(req.text)
        print("Text to speak: "+text)
        player=EspeakAudioPlayer()
        player.play_audio(text)
    elif req.command==3:#gpsr answer question
        xm = XM()

        # sent=xm.ASR_offline("question")
        sent=xm.ASR_online()
        sent=sent.strip().lower()
        #print(sent)

        xm.Questions(sent)
        xm_res.num=1
        xm_res.action.append("end")
        xm_res.object.append("question")
        xm_res.answer = xm.ans
        xm_res.gesture = xm.gesture
    elif req.command==4:#receptionist recognition
        xm = XM()
        # xm.name =""
        # xm.drink = ""
        # while xm.name == "":
        record = AudioSegment.from_wav("/home/xm/catkin_ws/src/speech/tts_record/receptionist_name.wav")
        play(record)
        sent = xm.ASR_online()
        #sent = xm.ASR_offline(" ")
        sent = sent.strip().lower()
        print("Recognized result: "+sent)
        xm.Receptionist(1,sent)
        # while xm.drink == "":
        record = AudioSegment.from_wav("/home/xm/catkin_ws/src/speech/tts_record/receptionist_drink.wav")
        play(record)
        sent = xm.ASR_online()
        #sent = xm.ASR_offline("gpsr")
        sent = sent.strip().lower()
        print("Recognized result: "+sent)
        xm.Receptionist(2,sent)

        xm_res.name=xm.name
        xm_res.drink=xm.drink
        print("name and drink to smach:ws_XfSpeech/devel/setup.bash ")
        print(xm_res.name, xm_res.drink)
    elif req.command==5:#shopping and tidy up and 通用机器人

        # sent="the apple is on the left"
        # sent="please give me the apple"
        # sent=xm.ASR_offline("gpsr")
        # sent=xm.ASR_offline(" ")
        
        sent=xm_globle.listener.listen_object().lower()
        print("Recognized result: "+sent)



        xm_globle.NLPnew(sent)

        print(xm_globle.num)
        print(xm_globle.act)
        print(xm_globle.obj)
        print(xm_globle.ans)
        print(xm_globle.gesture)

        xm_res.num=xm_globle.num
        xm_res.action=xm_globle.act
        xm_res.object=xm_globle.obj
        xm_res.answer=xm_globle.ans
        xm_res.gesture=xm_globle.gesture
        xm_globle.clear()
    elif req.command == 6:#yes or no
        xm = XM()
        sent = xm.ASR_online().lower()
        sent = sent.strip()
        print("Recognized result: " + sent)
        if "right" in sent:
            xm_res.gesture="yes"
        elif "wrong" in sent:
            xm_res.gesture = "no"
    elif req.command == 7:#garbage
        xm = XM()
        # sent = xm.ASR_offline(" ").lower()
        sent = xm.ASR_online().lower()

        sent = sent.strip()
        # sent="go to the dining room"
        print("Recognized result: " + sent)
        if "room" in sent:
            xm_res.num=1
            xm_res.action.append("go")
            if "bedroom" in sent:xm_res.object.append("bedroom")
            elif "dining" in sent:xm_res.object.append("dining_room")
            elif "living" in sent:xm_res.object.append("living_room")
        elif "kitchen" in sent:
            xm_res.num = 1
            xm_res.action.append("go")
            xm_res.object.append("kitchen")
        elif "corridor" in sent:
            xm_res.num = 1
            xm_res.action.append("go")
            xm_res.object.append("corridor")
    elif req.command==8:#中国机器人大赛-通用
        xm=XM()
        #sent="navigate to the kitchen, count the male people and report it to skyler which is in the diningroom"
        # sent = "navigate to the livingroom, look for a person and tell the sex of that person"
        #sent="count the male people"
        #sent="get the green tea from the living room"
        #sent="reach the diningroom, find a person and tell which gesture is performing that person"
        # sent="count the rising left arm people in the dining room"
        # sent="Tell me how many Apple are in the kitchen counter"
        # sent="mhm, my name is alex."
        # sent="Tell me how many apple are in the kitchen counter"
        # sent="count the apple at the kitchen counter and report to me"
        #sent=xm.ASR_offline("gpsr")
        # sent=xm.ASR_offline(" ")
        sent=xm.ASR_online().lower()
        # print(sent)
        # sent=sent.replace("robert, please","",1)
        # sent ='pick up the milk'
        sent=sent.strip()
        print("Recognized result: "+sent)

        n = xm.spacy_analyze(sent)

        #for nn in n:
        #    print(nn)
        print(" NLP start!")
        xm.NLP(sent)

        print("NLP over!")

        print(xm.num)
        print(xm.act)
        print(xm.obj)
        print(xm.ans)
        print(xm.gesture)

        for i in xm.obj :
            if i in objects:
                xm_res.object.append(i)

        xm_res.num=xm.num
        xm_res.action.append('go')

    else: print("Command Error!!!")

    

    return xm_res
    
def test():#IDE单独紧急测试使用
    xm=XM()
    #sent="take the napkins from the dining table and take it to daniel in the kitchen"
    # sent=xm.ASR_online().lower()
    sent = "the chocolate drink is on the left"
    print(sent)
    # n = xm.spacy_analyze(sent)
    # for nn in n:
    #     print(nn)
    xm.NLPnew(sent)
    print(xm.num)
    print(xm.act)
    print(xm.obj)
    print(xm.ans)

if __name__ == "__main__":
    # test()
    rospy.init_node("speech_core")
    s = rospy.Service('speech_core', speech_to_smach, call_back)
    rospy.spin()
    #test()











