#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine.h"
#include <string>
/*--- Índice --- 
(Usem Ctrl+F pra facilitar)
1.Organização de Dados: l12
2.Otimização: l77
3.Lógica Segue-Linha: l98
4.Lógica de Parada: l109
5.Leitura encoder: l39
*/
//1.Organização de Dados:
struct valsVel {
	float turnMin, turnMax, turnBase;
	float fwdMin, fwdMax, fwdBase;
	float motorLeft, motorRight;
};
struct valsPID {
	float inPID, outPID;
	float turnSp, turnKp, turnKi, turnKd;
	float fwdSp, fwdKp, fwdKi, fwdKd;
};
struct sensorVals {
	float min, max;
	float val[5], avg;
};
struct countVals {
	int left, right;
};
struct carVals {
	int state;//0 parado, 1 linha, 2  curva
	valsVel valVel;
	valsPID valPID;
	countVals sideMarks, motEncs;
	sensorVals sideSensor, arrSensor;
};
//------------------------------------------------

//5.Leitura encoder:
float fTime = 0;
int turns = 0;
float radius = 1;
float pi = 3.14159f;
float RPM = 0;
float distance = 0;
bool readDir = false;
int direction = 0;
void readEncoder(bool A, bool B, bool O, float elapsed) {
	turns += O;
	if (turns != 0) {
		if (readDir) {
			direction = (A) ? -1 : (B) ? 1 : 0;
			readDir = false;
		}
		readDir = A && B;//quando as entradas se sobreporem, leia a direção na proxima iteração
		//se rotação completa, recalcular RPM e reinicie o tempo
		if (O) {
			RPM = 1 / (fTime+(fTime==0));
			fTime = 0;
		}
		fTime += elapsed;
		distance = turns * RPM * (2 * radius * pi);
	}
}
//------------------------------------------------
class Simu {
public:
	//vars da linha
	olc::vi2d linePos;
	olc::vi2d lineSize = {40,30};//largura/altura da linha
	olc::vf2d pos = { 100,100 };//posição superior esquerda dos sensores
	int radius = 10;//raio de cada sensor

	int lookup[11] = { 31,16,24,8,12,4,6,2,3,1,0 };// 31 -> stopped // 0 -> no line
	carVals car;

	//2.Otimização:
	int errSum;
	int errPrev;
	void calcPID(float* input, float* output) {
		//se index for -5 ou 5, carro está parado/virando 180, mantenha PID
		if (std::abs(*input) == 5)return;
		int error = *input - 5;
		errSum += error;
		int errDiff = errPrev - error;
		*output = car.valPID.fwdKp * error + car.valPID.fwdKi * errSum + car.valPID.fwdKd * errDiff;
		errPrev = error;
	}
	//------------------------------------------------

	Simu() {
		car.valPID.fwdKp = 0.5f;
		car.valPID.fwdKi = 0.05f;
		car.valPID.fwdKd = 1.0f;
		car.sideMarks.right = 0;
	};

	//3.Lógica Segue-Linha:
	void run(olc::PixelGameEngine* pge) {
		readSensor(pge);
		calcPID(&car.valPID.inPID,&car.valPID.outPID);
		if (car.state > 0) drive();
		draw(pge);
	}
	void readSensor(olc::PixelGameEngine*pge) {
		//atualiza posição da linha
		linePos = { pge->GetMouseX() - 20, 10 };

		//4.Lógica de Parada:
		car.sideSensor.val[0] = pge->GetKey(olc::D).bPressed;//simulação da leitura do sensor lateral
		car.sideMarks.right += car.sideSensor.val[0];//se leu marca lateral, incrementa
		if (car.sideMarks.right > 1) {
			car.state = 0;
			car.sideMarks.right = 0;
		}
		//------------------------------------------------

		for (int i = 0; i < 5; i++) {
			//simulação dos sensores lendo a pista (Se pressionarem ou apertarem o botão esquerdo do mouse, todos os sensores serão ativados(1 1 1 1 1))
			float sPos = pos.x + 3 * radius * i;
			car.arrSensor.val[i] = (linePos.x < sPos && sPos <= linePos.x + lineSize.x) || (pge->GetMouse(0).bPressed || pge->GetMouse(0).bHeld);
		}
		//(re)definir indíce do erro como -1 (para checagem caso o valor lido não exista)
		car.valPID.inPID = -1;
		//leituras dos sensores são transformadas de binário para decimal, LSB na direita por padrão
		int in = 0;
		for (int i = 0; i < 5; i++) in += car.arrSensor.val[i] * (16 >> i);
		//esse loop procura pelo valor dentro da tabela de checagem, cujo indíce corresponde ao nível de erro
		for (int i = 0; i < 11; i++) car.valPID.inPID = (in == lookup[i]) ? i : car.valPID.inPID;
		//atualiza estado baseado no erro
		car.state = (car.valPID.inPID == 10) ? 0 : (car.valPID.inPID == 0) ? 2 : 1;
		car.valPID.inPID = (car.valPID.inPID == 10 || car.valPID.inPID == 0) ? 5 : car.valPID.inPID;
	}
	void drive() {
		car.valVel.motorLeft = car.valVel.fwdBase + car.valPID.outPID;
		car.valVel.motorRight = car.valVel.fwdBase - car.valPID.outPID;
		pos.x += 0.01f * (car.valVel.motorLeft - car.valVel.motorRight);
	}
	//------------------------------------------------

	void draw(olc::PixelGameEngine* pge) {
		pge->Clear(olc::GREY);
		pge->FillRect(linePos, lineSize, olc::BLACK);
		for (int i = 0; i < 5; i++) pge->FillCircle(pos.x + 3 * radius * i, pos.y, radius, (car.arrSensor.val[i]) ? olc::GREEN : olc::RED);
		pge->DrawString(pos, std::to_string(car.valPID.inPID - 5), olc::YELLOW);
		pge->DrawString(pos + olc::vi2d{ 75,20 }, "errSum: " + std::to_string(errSum), olc::YELLOW);
		pge->DrawString(pos + olc::vi2d{ 75,30 }, "errDiff: "+ std::to_string(errPrev-car.valPID.inPID-5), olc::YELLOW);
		pge->DrawString(pos + olc::vi2d{ 75,40 }, "errPrev: "+ std::to_string(errPrev), olc::YELLOW);
		pge->DrawString(pos + olc::vi2d{ 75,50 }, "outPID: " + std::to_string(car.valPID.outPID), olc::YELLOW);
	}
};

class Basics : public olc::PixelGameEngine
{
public:
	Basics()
	{
		sAppName = "Basic()";
	}
public:
	Simu simu;
	bool OnUserCreate() override
	{
		return true;
	}
	bool OnUserUpdate(float fElapsedTime) override
	{
		simu.run(this);
		return true;
	}
};

int main()
{
	Basics demo;
	if (demo.Construct(600, 400, 2, 2))
	{
		demo.Start();
	}
	return 0;
}

/* vvv COOL CODE vvv
* int sensors[5];
int lookup[11] = { 31,16,24,8,12,4,6,2,3,1,0 };// 31 -> stopped // 0 -> no line
int error = -1;
int in = 16 * sensors[0] + 8 * sensors[1] + 4 * sensors[2] + 2 * sensors[3] + sensors[4];
for (int i = 0; i < 11; i++) error = (in == lookup[i]) ? i : error;
*/

/* vvv PSYCHO CODE vvv
if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 1 )) error = 4;

else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 )) error = 3; 

else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 0 )) error = 2;

else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 0 )) error = 1;

else if((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) error = 0;

else if((LFSensor[0]== 0 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) error =- 1;

else if((LFSensor[0]== 0 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) error = -2;

else if((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) error = -3;

else if((LFSensor[0]== 1 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 )) error = -4;
*/