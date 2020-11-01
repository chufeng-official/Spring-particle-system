#include<graphics.h>
#include<math.h>
#include<Windows.h>
constexpr auto max_num_particles = 1024;
constexpr auto dt = 0.001;
constexpr auto substeps = 50;
constexpr auto e = 2.71828;
constexpr auto winWidth = 1000;
constexpr auto winHeight = 600;
constexpr auto connection_radius = 100;
constexpr auto particle_rest_length = 80;
constexpr auto radius = 8.0;

double spring_Y;
bool paused;
double drag_damping;
double dashpot_damping;
int num_particles;
int num_springs;
double g[2] = { 0,9.8 };
double x[max_num_particles][2];
double v[max_num_particles][2];
double a[max_num_particles][2];
int fixed[max_num_particles];
double rest_length[max_num_particles][max_num_particles];
double norm(double a[2])
{
	return sqrt(a[0] * a[0] + a[1] * a[1]);
}
double* normalized(double a[2])
{
	double norm_ = norm(a);
	double* b = (double*)malloc(2 * sizeof(double));
	if (!b)
		return NULL;
	*b = a[0] / norm_;
	*(b + 1) = a[1] / norm_;
	return b;
}
void substep()
{
	int n = num_particles;
	for (int i = 0; i < n; i++)
	{
		int n = num_particles;
		a[i][0] = g[0];
		a[i][1] = g[1];
		for (int j = 0; j < n; j++)
			if (rest_length[i][j])
			{
				double x_ij[2] = { x[i][0] - x[j][0],x[i][1] - x[j][1] };
				double* d = normalized(x_ij);
				double norm_ = norm(x_ij);
				a[i][0] += -spring_Y * (norm_ / rest_length[i][j] - 1) * d[0];
				a[i][1] += -spring_Y * (norm_ / rest_length[i][j] - 1) * d[1];
				double v_ij[2] = { v[i][0] - v[j][0],v[i][1] - v[j][1] };
				double v_rel = v_ij[0] * d[0] + v_ij[1] * d[1];
				a[i][0] += -dashpot_damping * v_rel * d[0];
				a[i][1] += -dashpot_damping * v_rel * d[1];
				free(d);
			}
	}
	for (int i = 0; i < n; i++)
	{
		if (!fixed[i])
		{
			v[i][0] += dt * a[i][0];
			v[i][1] += dt * a[i][1];
			v[i][0] *= pow(e, 0 - dt * drag_damping);
			v[i][1] *= pow(e, 0 - dt * drag_damping);
			x[i][0] += v[i][0] * dt;
			x[i][1] += v[i][1] * dt;
		}
		else
		{
			v[i][0] = 0;
			v[i][1] = 0;
		}
		if (x[i][0] - radius < 0)
		{
			x[i][0] = 0 + radius;
			v[i][0] = 0;
		}
		if (x[i][0] + radius > winWidth)
		{
			x[i][0] = winWidth - radius;
			v[i][0] = 0;
		}
		if (x[i][1] - radius < 0)
		{
			x[i][1] = 0 + radius;
			v[i][1] = 0;
		}
		if (x[i][1] + radius > winHeight)
		{
			x[i][1] = winHeight - radius;
			v[i][1] = 0;
		}
	}
}
void new_particle(int pos[2], int fixed_)
{
	int new_particle_id = num_particles;
	x[new_particle_id][0] = pos[0];
	x[new_particle_id][1] = pos[1];
	v[new_particle_id][0] = 0;
	v[new_particle_id][1] = 0;
	fixed[new_particle_id] = fixed_;
	num_particles += 1;
	for (int i = 0; i < new_particle_id; i++)
	{
		double x_ni[2] = { x[new_particle_id][0] - x[i][0],x[new_particle_id][1] - x[i][1] };
		double dist = norm(x_ni);
		if (dist < connection_radius)
		{
			rest_length[i][new_particle_id] = particle_rest_length;
			rest_length[new_particle_id][i] = particle_rest_length;
			num_springs += 1;
		}
	}
}
void attract(int pos[2])
{
	for (int i = 0; i < num_particles; i++)
	{
		v[i][0] += -dt * substeps * (x[i][0] - pos[0]) * 0.4;
		v[i][1] += -dt * substeps * (x[i][1] - pos[1]) * 0.4;
	}
}
int main(void)
{
	initgraph(winWidth, winHeight);
	setbkcolor(0xCCCCCC);
	setlinecolor(0x666666);
	setlinestyle(PS_SOLID, 2);
	settextcolor(0x000000);
	LOGFONT f;
	gettextstyle(&f);
	cleardevice();
	settextstyle(50, 0, L"黑体");
	wchar_t title[10] = L"弹簧质点系统";
	outtextxy(winWidth / 2 - textwidth(title) / 2, winHeight / 2 - textheight(title) / 2, title);
	settextstyle(&f);
	Sleep(1000);
	spring_Y = 500;
	drag_damping = 0.5; // 防抖
	dashpot_damping = 0.2; // 弹簧复原
	paused = false;
	MOUSEMSG m;
	int move = 1;
	while (1)
	{
		PeekMouseMsg(&m);
		int pos[2] = { m.x, m.y };
		switch (m.uMsg)
		{
		case WM_MOUSEMOVE:
			move = 1;
			break;
		case WM_LBUTTONUP:
			if(move)
				new_particle(pos, m.mkCtrl);
			move = 0;
			break;
		case WM_RBUTTONDOWN:
			if (m.mkCtrl)
			{
				if(move)
					paused = !paused;
				move = 0;
			}
			else
				attract(pos);
			break;
		case WM_MBUTTONUP:
			num_particles = 0;
			num_springs = 0;
			for (int i = 0; i < max_num_particles; i++)
				for (int j = 0; j < max_num_particles; j++)
					rest_length[i][j] = 0;
			break;
		case WM_MOUSEWHEEL:
			if (m.mkCtrl)
			{
				if(move)
					spring_Y *= pow(1.1, m.wheel / 120);
				move = 0;
			}
			break;
		}
		int n = num_particles;
		wchar_t numPar[10] = { 0 };
		_itow_s(n, numPar, 10);
		wchar_t message1[50] = L"左键生成质点，当前质点数： ";
		wcscat_s(message1, numPar);

		wchar_t numSpring[10] = { 0 };
		_itow_s(num_springs, numSpring, 10);
		wchar_t message3[50] = L"当前弹簧数： ";
		wcscat_s(message3, numSpring);

		wchar_t springY[20] = {0};
		_itow_s((int)spring_Y, springY, 10);
		wchar_t message2[50] = L"ctrl+滚轮设置杨氏模量的值，初始值为500，当前值为： ";
		wcscat_s(message2, springY);

		BeginBatchDraw();
		cleardevice();
		outtextxy(0 + 10, 0 + 10 + 25 * 0, message1);
		outtextxy(textwidth(message1) + 10 + 10, 0 + 10 + 25 * 0, message3);
		outtextxy(0 + 10, 0 + 10 + 25 * 1, L"右键生成引力中心");
		outtextxy(0 + 10, 0 + 10 + 25 * 2, L"中键清空屏幕");
		outtextxy(0 + 10, 0 + 10 + 25 * 3, L"ctrl+左键生成固定质点");
		outtextxy(0 + 10, 0 + 10 + 25 * 4, L"ctrl+右键暂停程序");
		outtextxy(0 + 10, 0 + 10 + 25 * 5, message2);
		if (paused)
		{
			wchar_t pausedMsg[5] = L"暂停中";
			settextcolor(RED);
			outtextxy(winWidth / 2 - textwidth(pausedMsg) / 2, winHeight / 2 - textheight(pausedMsg) / 2, pausedMsg);
			settextcolor(0x000000);
		}
		for (int i = 0; i < n; i++)
			for (int j = i + 1; j < n; j++)
				if (rest_length[i][j])
					line(x[i][0], x[i][1], x[j][0], x[j][1]);
		for (int i = 0; i < n; i++)
		{
			if (fixed[i])
				setfillcolor(RED);
			else
				setfillcolor(0x000000);
			solidcircle(x[i][0], x[i][1], radius);
		}
		EndBatchDraw();
		if (!paused)
			for (int step = 0; step < substeps; step++)
				substep();
	}
}
