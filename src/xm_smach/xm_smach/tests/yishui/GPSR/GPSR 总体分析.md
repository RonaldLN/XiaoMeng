# GPSR 总体分析

进入房间。由底盘判断門是否打开，若打开则比赛开始，机器人移动进入房间，初始房间为living room。

进入房间后，播放预先存好的音频文件，让人布置任务。（类似”give me the mission“）。

然后根据人的语音，接受任务。开启客户端，调用语音的服务，来获取处理后的任务信息。将信息输入到target、action和task_num中。action需要字符串数组，task_num是一项任务中的所有子动作个数。GPSR中应有三项任务。

判断下一步做什么。current_task初始值为-1，每次判断下一步做什么时，将其值+1。以current_task为下标找寻action中对应的字符串，判断当前应该做什么。共分为go、find、follow、get、tell、deliver。如果current_task数值等于总子动作数，则表明这一项任务完成，返回門处，听取下一项任务。

### 如果完成一项任务，回到門处。

判断是否完成所有任务。如果current_task数值等于3，则完成所有任务，出房间。

如果没有完成所有任务，则回到播放音频文件，让人布置一项任务。

### 如果还有子任务

根据action顺序，执行动作，每个action会对应一个target，用target确定需要移动到的位置。

