# -*- coding: utf-8 -*-
gestures={"waving":"waving","raising their left arm":"raising_their_left_arm","raising their right arm":"raising_their_right_arm",
"pointing to the left":"pointing_to_the_left","pointing to the right":"pointing_to_the_right","living":"waving","waiting":"waving","willing":"waving"}
poses={"sitting":"sitting","standing":"standing","lying down":"lying_down"}
rooms=["kitchen","living_room","dining_room","bedroom","corridor"]
rooms_new=["kitchen","living_room","dining_room","bedroom","corridor"]
oprop=["biggest","largest","smallest","heaviest","lightest","thinnest"]
beacons={"dishwasher":"kitchen","sink":"kitchen",
           "couch":"living_room","end_table":"living_room","bookcase":"living_room","entrance":"living_room",
           "dining_table":"dining_room","exit":"dining_room",
           "bed":"bedroom","desk":"bedroom"}
placements={"cupboard":"kitchen","sink":"kitchen","counter":"kitchen","storage_table":"kitchen",
           "end_table":"living_room","bookcase":"living_room",
           "dining_table":"dining_room",
           "side_table":"bedroom","desk":"bedroom"}
# #0-3:room1,4-6:room2,7-8:room3 location为大物体清单，value为对应所在的room
locations={"cupboard":"kitchen","dishwasher":"kitchen","sink":"kitchen","counter":"kitchen","storage_table":"kitchen",
           "couch":"living_room","end_table":"living_room","bookcase":"living_room","entrance":"living_room",
           "dining_table":"dining_room","exit":"dining_room",
           "bed":"bedroom","side_table":"bedroom","desk":"bedroom"}
people=[
#     "alex",
# "charlie",
# "elizabeth",
# "francis",
# "jennifer",
# "linda",
"mary",
# "patricia",
# "robin",
# "skyler",
# "james",
"john",
# "michael",
# "robert",
# "william"
    #     "me","him","her","person",
    #     "waving_person","raising_their_left_arm_person","raising_their_right_arm_person","pointing_to_the_left_person","pointing_to_the_right_person",
    #     "sitting_person","standing_person","lying_down_person",
    #     "man","woman","boy","girl","male_person","female_person",
    # "everyone","all_people","all_men","all_women","all_gusts","all_elders","all_children"
    ]


# #0-4:kitchen_counter,5-9:bookcase,10-12:dining_table objects为小物体清单，value为（未指定大物体时）的默认位置（大物体）
# objects={"milk":"kitchen_counter","cola":"kitchen_counter","water":"kitchen_counter","lemon_tea":"kitchen_counter","green_tea":"kitchen_counter",
#          "toothpaste":"bookcase","napkins":"bookcase","tea":"bookcase","apple":"bookcase","lemon":"bookcase",
#          "biscuits":"dining_table","chips":"dining_table","chocolates":"dining_table",
#          "person":"extra","question":"extra","it":"extra","objects":"extra",
#          "people":"","male_people":"","female_people":"","waving_people":"","rising_left_arm_people":"","rising_right_arm_people":"","pointing_left_people":"","pointing_right_people":""}
categories={"cleaning_stuff":"side_table","containers":"end_table","cutlery":"storage_table","drinks":"counter",
            "food":"cupboard","fruits":"bookcase","snacks":"bookcase","tableware":"storage_table"}
objects={"chocolate_drink":"drinks","coke":"drinks","grape_juice":"drinks","spring":"drinks","orange_juice":"drinks","sprite":"drinks","milk":"drinks",
         "noodles":"food","sausage":"food","gum":"food",
         "apple":"fruits","orange":"fruits",
         "pringle":"snacks","cracker":"snacks","potato_chips":"snacks",
        "my_bag":"luggage","baggage":"luggage","valise":"luggage","suitcase":"luggage","trolley":"luggage",
        "question":"","it":"","object":"","objects":"","soap":""
}
# chocolate_drink 元气森林乳茶
# sausage 香肠
# pringle 薯愿
# cracker 饼干
# noodles 汤达人
# milk    牛奶
# potato_chips 乐事薯片
# sprite  雪碧
# coke    可乐
# grape_juice 芬达葡萄汁
# apple   苹果
# orange  橙子
# orange_juice  果粒橙

# objects_new
objects_new={"cola":"drinks","orange_juice":"drinks","sprite":"drinks","water":"drinks",
         "chip":"snacks","lays":"snacks","cookie":"snacks",
         "handwash":"object","dishsoap":"object","shampoo":"object",
         "bread":"food","biscuit":"food","oreo":"snacks","potatowish":"snacks","milk":"drinks"
}
#take_o,take_p,bring_o,bring_p
actions={"get":"get","grasp":"get","bring_o":"get","take_o":"get","pick":"get","bring":"get",
        "put":"deliver","place":"deliver","give":"deliver","bring_p":"deliver","deliver":"deliver","serve":"deliver","arrange":"deliver","distribute":"deliver","provide":"deliver",
        "tell":"tell","say":"tell",
        "go":"go","navigate":"go",
        "find":"find","locate":"find","look_for":"find","contact":"find","face":"find","greet":"find",
        "guide":"guide","escort":"guide","take_p":"guide","lead":"guide","accompany":"guide",
        "follow":"follow",
        "answer":"answer",
        "stop":"stop",
        "meet":"meet",
        "ask":"ask_leave",
        "introduce":"introduce",
        "release":"release",
        "want":"want"
}
actions_new={
    "remember":"remember","memorize":"memorize",
    "stop":"stop",
    "follow":"follow",
    "give":"give","deliver":"give","get":"give","want":"give",
    "pick":"pick"
}
gesture_new=["left","right"]
questions={
"Who's the most handsome person in Canada":"I that Justin Trudeau is very handsome",
"How many time zones are there in Canada":"Canada spans almost 10 million square km and comprises 6 time zones",
"What's the longest street in the world":"Yonge Street in Ontario is the longest street in the world",
"How long is Yonge Street in Ontario":"Yonge street is almost 2,000 km, starting at Lake Ontario, and running north to the Minnesota border",
"What's the name of the bear cub exported from Canada to the London Zoo in 1915":"The bear cub was named Winnipeg. It inspired the stories of Winnie-the-Pooh",
"Where was the Blackberry Smartphone developed":"It was developed in Ontario, at Research In Motion's Waterloo offices",
"What is the world's largest coin":"The Big Nickel in Sudbury, Ontario. It is nine meters in diameter",
"In what year was Canada invaded by the USA for the first time":"The first time that the USA invaded Canada was in 1775",
"What year was Canada invaded by the USA for the second time":"The USA invaded Canada a second time in 1812",
"What country holds the record for the most gold medals at the Winter Olympics":"Canada does! With 14 Golds at the 2010 Vancouver Winter Olympics",
"Who coined the term Beatlemania":"Sandy Gardiner, a journalist of the Ottawa Journal",
"Why is Canada named Canada":"French explorers misunderstood the local native word Kanata, which means village",
"When was The Mounted Police formed":"The Mounted Police was formed in 1873",
"When was The Royal Canadian Mounted Police formed":"In 1920, when The Mounted Police merged with the Dominion Police",
"How big is the RCMP":"Today, the RCMP has close to 30,000 members",
"What else is Montreal called":"Montreal is often called the City of Saints or the City of a Hundred Bell Towers",
"Where is The Hotel de Glace located":"The Hotel de Glace is in Quebec",
"How many tons of ice are required to build The Hotel de Glace":"The Hotel de Glace requires about 400 tons of ice",
"How many tons of snow are required to build The Hotel de Glace":"Every year, 12000 tons of snow are used for The Hotel de Glace",
"Can I visit the Hotel de Glace in summer":"No. Every summer it melts away, only to be rebuilt the following winter",
"Where is Canada's only desert":"Canada's only desert is British Columbia",
"How big is Canada's only desert":"The British Columbia desert is only 15 miles long",
"Name 3 famous male Canadians":"Leonard Cohen, Keanu Reeves, and Jim Carrey",
"Name 3 famous female Canadians":"Celine Dion, Pamela Anderson, and Avril Lavigne",
"What's the origin of the Comic Sans font":"Comic Sans is based on Dave Gibbons' lettering in the Watchmen comic books",
"What is a nanobot":"The smallest robot possible is called a nanobot",
"How small can a nanobot be":"A nanobot can be less than one-thousandth of a millimeter",
"Why wasn't Tron nominated for an award by The Motion Picture Academy":"The Academy thought that Tron cheated by using computers",
"Which was the first computer with a hard disk drive":"he IBM 305 RAMAC",
"When was the first computer with a hard disk drive launched":"The IBM 305 RAMAC was launched in 1956",
"How big was the first hard disk drive":"The IBM 305 RAMAC hard disk weighed over a ton and stored 5 MB of data",
"What does CAPTCHA stands for":"CAPTCHA is an acronym for Completely Automated Public Turing test to tell Computers and Humans Apart",
"What was the first computer bug":"The first actual computer bug was a dead moth stuck in a Harvard Mark II",
"Name all of the robots on Mars":"There are four robots on Mars,Sojourner, Spirit, Opportunity, and Curiosity. Three more crashed on landing",
"What is a Mechanical Knight":"A robot sketch made by Leonardo DaVinci",
"Who is the world's first android":"Professor Kevin Warwick uses chips in his arm to operate doors, a robotic hand, and a wheelchair",
"What was the first computer in pass the Turing test":"Some people think it was IBM Watson, but it was Eugene, a computer designed at England's University of Reading",
"What does Moravec's paradox state":"Moravec's paradox states that a computer can crunch numbers like Bernoulli, but lacks a toddler's motor skills",
"What is the AI knowledge engineering bottleneck":"It is when you need to load an AI with enough knowledge to start learning",
"Why is Elon Musk is worried about AI's impact on humanity":"I don't know. He should worry more about the people's impact on humanity",
"Do you think robots are a threat to humanity":"No. Humans are the real threat to humanity",
"What is a chatbot":"A chatbot is an A.I. you put in customer service to avoid paying salaries",
"Are self-driving cars safe":"Yes. Car accidents are product of human misconduct",
"Who invented the compiler":"Grace Hoper. She wrote it in her spare time",
"Who created the C Programming Language":"C was invented by Dennis MacAlistair Ritchie",
"Who created the Python Programming Language":"Python was invented by Guido van Rossum",
"Is Mark Zuckerberg a robot":"Sure. I've never seen him drink water",
"Who is the inventor of the Apple I microcomputer":"My lord and master Steve Wozniak",
"Who is considered to be the first computer programmer":"Ada Lovelace",
"Which program do Jedi use to open PDF files":"Adobe Wan Kenobi"
}
# <pgenders>:"man" | "woman" | "boy" | "girl" | "male person" | "female person";
# <pgenderp>:"men" | "women" | "boys" | "girls" | "male" | "female";
# <oprop>:"biggest" | "largest" | "smallest" | "heaviest" | "lightest" | "thinnest";


# #category1下的动作清单，其value为该动作所属的类别
# action={"deliver":"deliver","take_to":"deliver","carry":"deliver","bring":"deliver",
#         "find":"find","look":"find",
#         "follow":"follow",
#         "go":"go","navigate":"go","reach":"go","get_into":"go",
#         "get":"get","take_from":"get","grasp":"get",
#         "speak":"speak","tell":"speak","say":"speak",
#         "stop":"stop",
#         "report":"report",
#         "answer":"answer",
#         "count":"count","counter":"count"}
# questions={
#     "which german count invented the zeppelin":"count von zeppelin",
#     "who was the first president of the usa":"george washington",
#     "in ancient china what meat was reserved for the emperor":"pork",
#     "in which city was the titanic built":"belfast",
#     "how many children did queen victoria have":"nine children",
#     "which french king was called the sun king":"louis xiv",
#     "what was in england the northern frontier of the roman empire":"hadrians wall",
#     "in which 1979 film was the spaceship called nostromo":"alien",
#     "who was the first king of belgium":"leopold i",
#     "what was the former name of new york":"new amsterdam",
#     "what was the latin name of paris in roman times":"lutetia",
#     "which city was the capital of australia from 1901 to 1927":"melbourne",
#     "give another name for the study of fossils":"paleontology",
#     "what do dragonflies prefer to eat":"mosquitoes",
#     "which insects cannot fly, but can jump higher than 30 centimeters":"fleas",
#     "what is the name of the european bison":"wisent",
#     "what is called a fish with a snake-like body":"eel fish",
#     "which plant does the canadian flag contain":"maple",
#     "which is the largest species of the tiger":"siberian tiger",
#     "which malformation did marilyn monroe have when she was born":"six toes",
#     "what is the house number of the simpsons":"number 742",
#     "what dog in ancient china was restricted to the aristocracy":"pekinese",
#     "who is the director of reservoir dogs":"quentin tarantino",
#     "what number is on herbie the beatle":"fifty-three",
#     "give the name of the best james bond parody":"austin powers",
#     "what is the name of the bald commander of the enterprise in star trek":"captain picard",
#     "who was the leading actress in sister act 1 en 2 ":"whoopi goldberg",
#     "what is the country top-level domain of belgium":"the .be domain",
#     "which unit is an indication for the sound quality of mp3":"kbps",
#     "in computing what is ram short for":"random access memory",
#     "who was the first man to fly around the earth with a spaceship":"yuri gagarin",
#     "on which hemisphere were the most dinosaur skeletons found":"the northern hemisphere",
#     "what color is cobalt":"blue",
#     "who invented vulcanized rubber":"goodyear",
#     "which device do we use to look at the stars":"telescope",
#     "which unit indicates the light intensity":"candela",
#     "who invented the barometer":"torricelli",
#     "who was the first american in space":"alan shepard",
#     "two brothers invented the hot air balloon. what was their surname":"montgolfier",
#     "who was the inventor of the steam engine":"james watt",
#     "which device was invented by henry mill":"the typewriter",
#     "what is the lightest existing metal":"aluminium",
#     "what are the three primary colors":"blue, yellow and red",
#     "which planet is nearest the sun":"mercury",
#     "how long is the great wall of china":"6259 kilometers",
#     "what is the largest number of five digits":"99999",
#     "what is the most fractured human bone":"clavicle",
#     "which south american country is named after venice":"venezuela",
#     "how many stars feature on the flag of new zealand":"four stars",
#     "what colour to do you get when you mix red and white":"pink"
# }
