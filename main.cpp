/* 
 * 编译运行前须安装easyx图形库
 * 编译环境：Visual C++ 6.0、Visual Studio 2008 ~ 2019 (x86 & x64) 
 */
#include<time.h>
#include<graphics.h>
#include<math.h>
#define vector(x,y) {x[0] - y[0], x[1] - y[1]} // 求两点间的向量
#define norm(x) sqrt(x[0] * x[0] + x[1] * x[1]) // 求模
#define normalize(x) { x[0] / norm(x), x[1] / norm(x) } // 求单位向量
#define dotProduct(x,y) (x[0] * y[0] + x[1] * y[1]) // 求内积
const int maxNumParticles = 1024; // 最大质点数
const double dt = 1e-3; // 演化时间间隔
const int evolutionsNum = 200; // 演化次数
const int connectionRadius = 100; // 两质点连接阈值
const int SpringOriginalLength = 80; // 弹簧原长
const double radius = 5.0; // 质点半径
const double mass = 1; // 质点质量
const int winWidth = 1000; // 窗口宽度
const int winHeight = 600; // 窗口高度
const int margin = 10; // 文字段落间隔，文字与屏幕边缘间隔
const double mouseAttractModulus = 0.2; // 鼠标右键吸引系数
const int forceAnalysisModulus = 20000; // 受力示意系数，越小越容易变红
const int maxRunNum = 100; // 检测FPS的循环次数
const int TextMessageNum = 11; // 屏幕左上方说明文字的条数
const int ModulusMessageNum = 5; // 屏幕左边系数文字的条数
const wchar_t TextMessageList[TextMessageNum][50] = { // 存储屏幕右上方说明文字
		L"点击鼠标左键生成一个质点",
		L"按住鼠标右键生成引力中心",
		L"点击鼠标中键撤销上一步质点的生成",
		L"ctrl+左键生成一个固定质点",
		L"ctrl+右键暂停程序",
		L"ctrl+中键清空物体",
		L"shift+右键开启或关闭受力示意",
		L"ctrl+滚轮设置劲度系数的值，初始值为10",
		L"shift+滚轮的操作分以下两种情况：",
		L"若鼠标指针在屏幕左半边，则设置空气阻力系数值，初始值为100",
		L"若鼠标指针在屏幕右半边，则设置减震系数值500"
	};
const wchar_t ModulusMessageList[ModulusMessageNum][20] = { // 储存系数的提示信息
	L"当前质点数：",
	L"当前弹簧数：",
	L"当前劲度系数值：",
	L"当前空气阻力系数值：",
	L"当前减震系数值："
};
double k; // 劲度系数
bool paused; // 是否暂停演化
bool forceAnalysis; // 是否开启受力示意功能
int mousePos[2]; // 鼠标位置
MOUSEMSG m; // 鼠标事件结构体
int runNum, FPS; // 记录当前循环次数和每秒循环次数
clock_t startTime, endTime; // 记录时间，用于计算FPS
int atmosphericDrag; // 空气阻力系数
int shockAbsorptionFactor;// 减震系数
int particlesNum; // 当前质点数
int springsNum; // 当前弹簧数
double gravity[2] = { 0 * mass, 9.8 * mass}; // 重力加速度
typedef struct particle // 质点对象
{
	double x[2]; // 位置矢量
	double v[2]; // 速度矢量
	double f[2]; // 合外力矢量（仅包含弹簧力）
	bool fixed;  // 是否固定
	bool connected[maxNumParticles]; // 与其他质点的距离
} PARTICLE;
PARTICLE particles[maxNumParticles]; // 定义数组，用来储存所有质点
void CalculateIndividualForce(int i,int j) // 计算第i，j个质点间的弹簧力
{
	double x_ij[2] = vector(particles[i].x, particles[j].x); // x_ij为从i到j的向量
	double unitVector[2] = normalize(x_ij); // 求x_ij的单位向量，为后续确定这两点间的方向
	double normOfx_ij = norm(x_ij); // 求x_ij的模，即两点间的距离
	particles[i].f[0] += -k * (normOfx_ij - SpringOriginalLength) * unitVector[0]; // 胡克定律求弹簧力
	particles[i].f[1] += -k * (normOfx_ij - SpringOriginalLength) * unitVector[1]; // 胡克定律求弹簧力
}
void CalculateAllSpringForceOfTheParticle(int id) // 计算第id个质点所受的所有弹簧力的合力
{
	particles[id].f[0] = 0;
	particles[id].f[1] = 0;
	/* 
	 * 注：力与速度和位移不同，
	 * 速度和位移由上一帧的速度和位移以及这一帧的受力算出，
	 * 而力只与这一帧的受力情况有关，不可与上一帧的力叠加，
	 * 所以要重新初始化为0，否则会导致严重抽搐
	 */ 
	for (int j = 0; j < particlesNum; j++) // 研究第j个质点与第i个质点的相互作用
		if (particles[id].connected[j]) // 如果两质点连接
			CalculateIndividualForce(id, j); // 计算第i，j个质点间的弹簧力
}
void captiveTheParticle(int id) // 防止质点超越四面墙壁
{
	if (particles[id].x[0] - radius < 0) // 左墙
	{
		particles[id].x[0] = radius;
		particles[id].v[0] = 0;
	}
	if (particles[id].x[0] + radius > winWidth) // 右墙
	{
		particles[id].x[0] = winWidth - radius;
		particles[id].v[0] = 0;
	}
	if (particles[id].x[1] - radius < 0) // 天花板
	{
		particles[id].x[1] = radius;
		particles[id].v[1] = 0;
	}
	if (particles[id].x[1] + radius > winHeight) // 地板
	{
		particles[id].x[1] = winHeight - radius;
		particles[id].v[1] = 0;
	}
}
void CalculateTheVXOfTheParticle(int id) // 计算一个质点下一隐式帧的速度和位移
{
	if (!particles[id].fixed) // 如果该质点不固定
	{
		particles[id].v[0] += dt * (particles[id].f[0] + gravity[0]) / mass; // v = v0 + F / m * dt 
		particles[id].v[1] += dt * (particles[id].f[1] + gravity[1]) / mass; // 速度公式
		particles[id].v[0] *= 1 / (atmosphericDrag * dt * 1e-3 + 1); // 空气阻力
		particles[id].v[1] *= 1 / (atmosphericDrag * dt * 1e-3 + 1);
		for (int j = 0; j < particlesNum; j++)
		{
			if (particles[id].connected[j])
			{
				double x_ij[2] = vector(particles[id].x, particles[j].x); // x_ij为从i到j的向量
				double unitVector[2] = normalize(x_ij); // 求x_ij的单位向量，为后续确定这两点间的方向
				double v_ij[2] = vector(particles[id].v, particles[j].v); // 求两点间的相对速度，便于计算阻尼
				double v_rel = dotProduct(v_ij, unitVector); // 将v_ij向量投影到单位向量上得到的长度
				particles[id].v[0] += -shockAbsorptionFactor * v_rel * unitVector[0] * dt * 1e-3; // 减震阻尼
				particles[id].v[1] += -shockAbsorptionFactor * v_rel * unitVector[1] * dt * 1e-3; // 减震阻尼
			}
		}
		particles[id].x[0] += particles[id].v[0] * dt; // x = x0 + v * dt
		particles[id].x[1] += particles[id].v[1] * dt; // 位移公式
	}
	else
	{
		particles[id].v[0] = 0;
		particles[id].v[1] = 0;
		/* 
		 * 虽然是全局变量，初始速度为0
		 * 但是鼠标拖拽会影响速度
		 * 所以每一帧都要再固定一下，防止移动
		 */
	}
	captiveTheParticle(id); // 防止质点超越四面墙壁
}
void updateData() // 更新下一隐式帧
{
	for (int i = 0; i < particlesNum; i++)// 遍历质点列表
		CalculateAllSpringForceOfTheParticle(i); // 计算质点所受的所有弹簧力的合力
	for (int i = 0; i < particlesNum; i++) // 遍历质点列表
		CalculateTheVXOfTheParticle(i); // 计算质点下一隐式帧的速度和位移
}
void addNewSpring(int i, int j) // 在第i，j个质点间加上弹簧
{
	particles[i].connected[j] = true; // 第i个质点与新质点连接
	particles[j].connected[i] = true; // 新质点与第i个质点连接
	springsNum += 1; // 弹簧数加一
}
void appendNewParticle(int pos[2], bool fixity) // 添加一个新质点
{
	int newId = particlesNum; // 新质点的索引正好为当前质点数
	particles[newId].x[0] = pos[0]; // 新质点位置为鼠标位置
	particles[newId].x[1] = pos[1]; 
	particles[newId].fixed = fixity; // 是否固定
	particlesNum += 1; // 总质点数加一
	for (int i = 0; i < newId; i++) // 以新质点为研究对象
	{
		double x_ni[2] = vector(particles[newId].x, particles[i].x); // 新质点与第i个质点之间的向量
		if (norm(x_ni) < connectionRadius) // 判断距离是否可以连接弹簧
			addNewSpring(i,newId); // 在第i，j个质点间加上弹簧
	}
}
void attractObject(int pos[2]) // 鼠标右键吸引效果
{
	for (int i = 0; i < particlesNum; i++) // 遍历所有质点
	{
		particles[i].v[0] += dt * evolutionsNum * (pos[0] - particles[i].x[0]) * mouseAttractModulus; // 横坐标加速
		particles[i].v[1] += dt * evolutionsNum * (pos[1] - particles[i].x[1]) * mouseAttractModulus; // 纵坐标加速
		// 注：不要试图使用万有引力公式，否则会变成一团糟，远的吸引不过来，近的抽搐得飞快
	}
}
void showInt(int posX, int posY, const wchar_t* message, int num) // 在屏幕上（posX，posY）的位置显示说明文字message和整数num
{
	wchar_t numString[20] = L""; // 定义字符串numString用于储存num
	_itow_s(num, numString, 10); // 把num转化为字符串后存入numString
	wchar_t caption[20] = L""; // 定义字符串caption用于储存message
	wcscat_s(caption, 20, message); // 把message存入caption
	wcscat_s(caption, 20, numString); // 再把numString拼接到caption之后
	outtextxy(posX, posY, caption); // 在屏幕上（posX，posY）的位置显示caption
}
void removeSpring(int i, int j) // 删除第i，j个质点间的弹簧
{
	particles[i].connected[j] = false; // 取消连接
	particles[j].connected[i] = false; // 取消连接
	springsNum--; // 弹簧数减一
}
void undo() // 撤销
{
	if (particlesNum > 0) // 撤销不能让总质点数小于零
	{
		particlesNum--; // 总质点数减一
		for (int i = 0; i < particlesNum; i++) // 遍历所有质点
			if (particles[particlesNum].connected[i]) // 如果最后放置的质点与第i个质点有连接
				removeSpring(i, particlesNum); // 删除第i，j个质点间的弹簧
		/* 
		 * 不必让 particles[particlesNum] = 0 ，
		 * 这是因为本程序所有的遍历质点列表的操作都是只遍历到particlesNum
		 * 不会去读取索引大于particlesNum的废弃质点的数据，
		 * 而当添加新质点时，这些被撤销的旧数据就会被覆盖，不会影响程序正常运行
		 */
	}
}
void clearAllData() // 清除所有质点和弹簧
{
	while (particlesNum) // 所谓清空就是不断撤销，直到总质点数 particlesNum == 0
		undo();
}
void mouseMoveEvents() // 鼠标移动事件
{
	if (m.mkRButton) // 如果移动的时候使用鼠标右键，则可以产生吸引效果
		attractObject(mousePos);
	/* 
	 * 注：这里是与单独鼠标右键点击的效果是一样的，
	 * 然而在单独检测鼠标右键的时候，移动鼠标就会覆盖右键事件的探测，
	 * 所以无法同时移动鼠标和按下右键，只好在检测鼠标移动事件这里再写一次吸引效果
	 */
}
void leftButtonUpEvents() // 左键松开事件
{
	appendNewParticle(mousePos, m.mkCtrl); // 添加新质点
	m.uMsg = false; // 清空事件结构体
}
void rightButtonDownEvents() // 右键按下事件
{
	if (m.mkCtrl)
	{
		paused = !paused; // 右键+ctrl，暂停
		m.uMsg = false; // 清空事件结构体
	}
	else if (m.mkShift)
	{
		forceAnalysis = !forceAnalysis; // 右键+shift，开启受力示意
		m.uMsg = false; // 清空事件结构体
	}
	else
		attractObject(mousePos); // 仅按右键则产生吸引效果
}
void middleButtonUpEvents() // 中键松开事件
{
	if (m.mkCtrl)
		clearAllData(); // 中键+ctrl，清除所有质点和弹簧
	else
		undo(); // 仅按中键撤销上一步
	m.uMsg = false; // 清空事件结构体
}
void mouseWheelEvents() // 鼠标滚轮事件
{
	if (m.mkCtrl)
	{
		k *= pow(1.2, m.wheel / 120); // ctrl+滚轮，增加或减少弹簧的劲度系数
		/* 
		 * 如果用 k *= 1.2，那么只能增大
		 * 而 m.wheel 总是120的倍数，并且每次循环都会清空这个值，
		 * 所以 m.wheel 会一直保持在120
		 * 所以用 k *= pow(1.2, m.wheel / 120) 就保证了：
		 * 滚轮正向滚动的时候，m.wheel ==  120，此时 k *= 1.2；
		 * 滚轮反向滚动的时候，m.wheel == -120，此时 k /= 1.2；
		 */
	}
	else if (m.mkShift) // shift+滚轮
	{
		if(m.x < winWidth / 2) // 如果鼠标在屏幕左半边，那么设置空气阻力系数
			atmosphericDrag *= pow(1.2, m.wheel / 120);
		else // 如果鼠标在屏幕右半边，那么设置减震系数
			shockAbsorptionFactor *= pow(1.2, m.wheel / 120);
	}
	m.uMsg = false; // 清空事件结构体
}
void getMouseEvents() // 获取鼠标事件并执行相关函数
{
	PeekMouseMsg(&m); // 检测鼠标事件
	mousePos[0] = m.x; // 存入鼠标位置
	mousePos[1] = m.y; // 存入鼠标位置
	switch (m.uMsg) // 判断鼠标事件
	{
	case WM_MOUSEMOVE: // 鼠标移动事件
		mouseMoveEvents();
		break;
	case WM_LBUTTONUP: // 左键松开事件
		leftButtonUpEvents();
		break;
	case WM_RBUTTONDOWN: // 右键按下事件
		rightButtonDownEvents();
		break;
	case WM_MBUTTONUP: // 中键松开事件
		middleButtonUpEvents();
		break;
	case WM_MOUSEWHEEL: // 鼠标滚轮事件
		mouseWheelEvents();
		break;
	}
}
void detectFPS() // 探测FPS
{
	if(!runNum) // 如果runNum == 0,则记录起始时刻
		startTime = clock(); // 记录起始时刻
	else if (runNum == maxRunNum - 1) // 如果runNum == 99，则再次记录当前时间
	{
		endTime = clock(); // 记录结束时刻
		double totalTime = (double)((double)endTime - (double)startTime) / CLOCKS_PER_SEC; // 时刻差，也就是100次循环所用总时间，单位：秒
		FPS = maxRunNum / totalTime; // 根据100次循环所用的总时间算每秒循环的次数，也就是FPS
		runNum = -1; // 再次初始化runNum为0，由于下面要自增，所以此处初始化为-1
	}
	runNum++; // 自增，直到为99时再次执行上述程序
}
void drawMainMessages() // 在屏幕上绘制所有的文字信息
{
	int i;
	int modulusList[ModulusMessageNum] = { // 储存要绘制在屏幕左边的系数
		particlesNum,
		springsNum,
		k,
		atmosphericDrag,
		shockAbsorptionFactor
	};
	for(i = 0; i < TextMessageNum; i++)
		outtextxy(margin, margin + (textheight(TextMessageList[i]) + margin) * i, TextMessageList[i]); // 绘制说明文字
	i++; // 空出一行
	for(int j = 0; j < ModulusMessageNum; j++)
		showInt(margin, margin + (textheight(ModulusMessageList[j]) + margin) * (i + j), ModulusMessageList[j], modulusList[j]); // 绘制系数信息
	showInt(winWidth - margin - textwidth(L"FPS：") - 20, margin, L"FPS：", FPS); // 绘制FPS
}
void drawPausedMessage() // 暂停时在屏幕中央绘制“暂停中”的红字
{
	wchar_t pausedMsg[5] = L"暂停中";
	settextcolor(RED);
	outtextxy(winWidth / 2 - textwidth(pausedMsg) / 2, winHeight / 2 - textheight(pausedMsg) / 2, pausedMsg);
	settextcolor(0);
}
void drawPolylines(int id) // 绘制辅助线
{
	setlinestyle(PS_DASHDOTDOT, 3);
	setlinecolor(0xAAAAAA);
	line(particles[id].x[0], particles[id].x[1], mousePos[0], mousePos[1]);
}
void drawTheSpring(int i, int j) // 绘制第i，j个质点之间的弹簧
{
	setlinestyle(PS_SOLID, 2);
	if (forceAnalysis) // 如果开启了受力示意功能。弹簧力越小，颜色越绿；弹簧力越大，颜色越红
	{
		int R = 256.0 * 256.0 * norm(particles[i].f) / (forceAnalysisModulus + 256 * norm(particles[i].f));
		// 使用 256*256*x/(λ + 256*x)函数，这样红色值在力为0时为0，在力趋近于无穷时趋近于256，这样就避免了溢出问题，并且过度自然
		int G = 255 - R; // 跟红色值相反
		setlinecolor(RGB(R, G, 0)); // 设置颜色，蓝色就让它为0算了
	}
	else
		setlinecolor(0x666666); // 不开就设置弹簧为深灰色
	line(particles[i].x[0], particles[i].x[1], particles[j].x[0], particles[j].x[1]);
}
void drawLines(int id)
{
	for (int j = id + 1; j < particlesNum; j++)
		if (particles[id].connected[j]) // 如果第id，j个质点之间连了弹簧
			drawTheSpring(id, j); // 则绘制弹簧
	double temp[2] = vector(particles[id].x, mousePos); // 鼠标位置指向周围质点的向量
	// 假设鼠标位置有一个质点，绘制这个质点与周围可连接的质点间的虚拟弹簧，也就是辅助线
	if (norm(temp) < connectionRadius)
		drawPolylines(id); // 绘制辅助线
}
void drawTheParticle(int id) // 绘制质点
{
	if (particles[id].fixed) // 如果是固定的，就设置为红色
		setfillcolor(RED);
	else
		setfillcolor(0); // 否则就是黑色
	solidcircle(particles[id].x[0], particles[id].x[1], radius);
}
void evolve() // 更新下一显式帧
{
	for (int i = 0; i < evolutionsNum; i++) // 更新 evolutionsNum 次隐式帧
		updateData(); // 更新下一隐式帧
}
int main(void)
{
	initgraph(winWidth, winHeight); // 初始化图形界面
	setbkcolor(0xCCCCCC); // 设定背景色为淡灰色
	settextcolor(0); // 设置字体颜色为黑色，这是因为字体默认为白色，在淡灰色背景上看不清
	k = 10; // 弹簧默认劲度系数
	atmosphericDrag = 100; // 空气阻力系数
	shockAbsorptionFactor = 500; // 减震系数
	while (true) // 开始演化主循环
	{
		detectFPS(); // 检测FPS
		getMouseEvents(); // 获取鼠标事件并执行相关函数
		BeginBatchDraw(); // 开始批量绘图，也就是直到 EndBatchDraw() 再一并显示所用的图形，解决频闪问题
		cleardevice(); // 清空屏幕的所有图形，防止影响下一帧
		drawMainMessages(); // 绘制所有文字信息
		for (int i = 0; i < particlesNum; i++) // 遍历所有质点
		{
			drawLines(i); // 绘制第i个质点周围的所有线
			drawTheParticle(i); // 绘制第i个质点
		}
		if (paused) // 如果暂停
			drawPausedMessage();// 绘制暂停信息
		else // 如果不暂停
			evolve(); // 更新下一显式帧
		EndBatchDraw(); // 结束批量绘图
	}
}
