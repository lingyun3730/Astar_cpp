/*
�㷨˵��/α���룺
����ʼ����ӵ�"�����б�
do
{
	Ѱ�ҿ����б���Fֵ��͵ĸ��ӣ����ǳ���Ϊ��ǰ����
	�����л����ر��б�
	���ڵ�ǰ�����ڵ�8�����ӵ�ÿһ��
		if(������ͨ�� || �Ѿ��ڡ��ر��б��У�
		{
			ʲôҲ������
		}
		if(�����ڿ����б��У�
		{
			������ӽ��������б����ѵ�ǰ����Ϊ��һ��ĸ��ڵ㣬������һ���FGH��
		}
		if(���Ѿ��ڿ����б��У�
		{
			if(��Gֵ��Ϊ�ο�����µ�·���ǲ��Ǹ��ã����͵�Gֵ��ζ�Ÿ��õ�·����
			{
				����һ��ĸ��ڵ�ĳɵ�ǰ�񣬲������¼�����һ���GFֵ��
			}
		}
} while(�����б��ǿյ��һ�û�ҵ�Ŀ��ڵ㣩;
��������б��ǿյģ�˵��·�������ڣ�

������Ŀ����ӿ�ʼ������ÿһ�����ڵ��ƶ�ֱ���ص���ʼ���ӣ������·����
*/

#include<math.h>
#include"Astar.h"

void Astar::InitAstar(std::vector<std::vector<int>> &_maze)
{
	maze = _maze;
}
//��������ʼ��ľ���
int Astar::calcG(Point *temp_start, Point *point)
{
	//���㵱ǰ�ڵ㵽�丸�ڵ�ľ��룬�����ڸ��ڵ�ĶԽ��߻�������������
	int extraG = (abs(point->x - temp_start->x) + abs(point->y - temp_start->y)) == 1 ? kCost1 : kCost2;
	//����ǳ�ʼ�ڵ㣬���丸�ڵ��ǿյģ����㸸�ڵ㵽��ʼ��ľ���
	int parentG = point->parent == NULL ? 0 : point->parent->G;
	return parentG + extraG;
}

//�������յ�Ĺ��ƾ���
int Astar::calcH(Point *point, Point *end)
{
	//�ü򵥵�ŷ�Ｘ�¾������H�����H�ļ���ʱ�ؼ���H��ֵ��С�ڵ���ʵ�ʵľ���ģ���˿����ҵ���̾���
	return (int) (sqrt((double)(end->x - point->x)*(double)(end->x - point->x) + (double)(end->y - point->y)*(double)(end->y - point->y)) * kCost1);
}

int Astar::calcF(Point *point)
{
	return point->G + point->H;
}

//�ҳ������б��е�Fֵ��С�Ľڵ�
Point *Astar::getLeastFpoint()
{
	if (!openList.empty())
	{
		auto resPoint = openList.front();
		for (auto &point : openList)
		{
			if (point->F < resPoint->F)
				resPoint = point;
		}
		return resPoint;
	}
	return NULL;
}
//isIgnoreCorner�Ǳ�־λ�����ڴ����Ƿ������ǣ�false��ʾ�������ǣ����ǲ���ͨ����true��ʾ���������ǣ�������ͨ��
Point *Astar::findPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner)
{
	//����������㣬��������һ���ڵ㣬�������
	openList.push_back(new Point(startPoint.x, startPoint.y));
	while (!openList.empty())
	{
		auto curPoint = getLeastFpoint(); //�ҵ�Fֵ��С�ĵ�
		openList.remove(curPoint); //�ӿ����б���ɾ��
		closeList.push_back(curPoint); //�ŵ��ر��б�
		//1.�ҵ���ǰ��Χ�˸������п���ͨ���ĸ���
		auto surroundPoints = getSurrondPoints(curPoint, isIgnoreCorner);
		for (auto &target : surroundPoints)
		{
			//����ĳ�����ӣ���������ڿ����б��У����뵽�����б����õ�ǰ����Ϊ�丸�ڵ㣬����FGH
			if (!isInList(openList, target))
			{
				//target�ĸ��ڵ�����ΪcurPoint
				target->parent = curPoint;
				target->G = calcG(curPoint, target);
				target->H = calcH(target, &endPoint);
				target->F = calcF(target);
				openList.push_back(target);
			}
			//����һ�����ӣ����ڿ����б��У�����Gֵ�������ԭ���Ĵ󣬾�ʲô�������������������ĸ��ڵ�Ϊ��ǰ�ڵ㣬��������G��F
			else
			{
				//tempG�Ǿ�����ǰ�ڵ��Gֵ
				int tempG = calcG(curPoint, target);
				//������ǰ�ڵ��Gֵ��ԭ����GֵС����ʾ������ǰ�ڵ��ǽ�Ϊ�ŵ�·������Ҫ���и���GHF
				if (tempG < target->G)
				{
					target->parent = curPoint;
					target->G = tempG;
					target->F = calcF(target);
				}
			}
			//�ж��յ�endPoint�Ƿ���openList��
			Point *resPoint = isInList(openList, &endPoint);
		    //����յ�endPoint����openList�У���ô�����յ�����
			if (resPoint)
			{
				return resPoint; //�����б���Ľڵ�ָ�룬��Ҫ��ԭ�������endpointָ�룬��Ϊ���������
			}
		}
	}
	//openListΪ�ն���resPoint�������У�����NULL
	return NULL;
}

//����·�����㷨
std::list<Point *> Astar::GetPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner)
{
	//�õ����սڵ�
	Point *result = findPath(startPoint, endPoint, isIgnoreCorner);
	std::list<Point *> path;
	//����·����·������˫������List����ʽ���ڵ�
	while (result)
	{
		path.push_back(result);
		result = result->parent;
	}
	return path;
}

//�ж�point�ǲ������б�list��
Point *Astar::isInList(const std::list<Point*> &list, const Point *point) const
{
	for (auto p : list)
	{
		if (p->x == point->x && p->y == point->y)
		{
			return p;
		}
	}
	return NULL;
}

bool Astar::isCanreach(const Point *point, const Point *target, bool isIgnoreCorner) const
{
	//������뵱ǰ�ڵ��غϣ�������ͼ�����ϰ�������ڹر��б��У�����false,��ʾ��ǰ�ڵ㲻��������һ���ж�
	if (target->x<0 || target->x>maze.size() - 1 || target->y<0 || target->y>maze[0].size() - 1 || maze[target->x][target->y] == 1 || (target->x == point->x &&target->y == point->y) || isInList(closeList, target))
		return false;
	else
	{
		if (abs(point->x - target->x) + abs(point->y - target->y) == 1) //����б��
			return true;
		else
		{
			//б�Խ�Ҫ�ж��Ƿ񱻰�ס
			if (maze[point->x][target->y] == 0 && maze[target->x][point->y] == 0)
				return true;
			else
				return isIgnoreCorner; //������õ��Ǻ��԰�ס���������Ȼ�п��ܷ���true
		}
	}
}

std::vector<Point *> Astar::getSurrondPoints(const Point *point, bool isIgnoreCorner) const
{
	std::vector<Point *> surroundPoints;
	for (int x = (point->x - 1); x <= (point->x + 1);x++)
		for (int y = (point->y - 1); y <= (point->y + 1);y++)
			if (isCanreach(point, new Point(x, y), isIgnoreCorner))
				surroundPoints.push_back(new Point(x, y));
	return surroundPoints;
}