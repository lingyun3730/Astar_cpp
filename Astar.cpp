/*
算法说明/伪代码：
把起始格添加到"开启列表“
do
{
	寻找开启列表中F值最低的格子，我们称它为当前格子
	把它切换到关闭列表。
	对于当前格相邻的8个格子的每一个
		if(它不可通过 || 已经在”关闭列表“中）
		{
			什么也不做；
		}
		if(它不在开启列表中）
		{
			把它添加进“开启列表”，把当前格作为这一格的父节点，计算这一格的FGH；
		}
		if(它已经在开启列表中）
		{
			if(用G值作为参考检查新的路径是不是更好，更低的G值意味着更好的路径）
			{
				把这一格的父节点改成当前格，并且重新计算这一格的GF值。
			}
		}
} while(开启列表不是空的且还没找到目标节点）;
如果开启列表是空的，说明路径不存在；

从最后的目标格子开始，沿着每一个父节点移动直到回到起始格子，这就是路径。
*/

#include<math.h>
#include"Astar.h"

void Astar::InitAstar(std::vector<std::vector<int>> &_maze)
{
	maze = _maze;
}
//计算离起始点的距离
int Astar::calcG(Point *temp_start, Point *point)
{
	//计算当前节点到其父节点的距离，看其在父节点的对角线还是在上下左右
	int extraG = (abs(point->x - temp_start->x) + abs(point->y - temp_start->y)) == 1 ? kCost1 : kCost2;
	//如果是初始节点，则其父节点是空的，计算父节点到起始点的距离
	int parentG = point->parent == NULL ? 0 : point->parent->G;
	return parentG + extraG;
}

//计算离终点的估计距离
int Astar::calcH(Point *point, Point *end)
{
	//用简单的欧里几德距离计算H，这个H的计算时关键，H的值是小于等于实际的距离的，因此可以找到最短距离
	return (int) (sqrt((double)(end->x - point->x)*(double)(end->x - point->x) + (double)(end->y - point->y)*(double)(end->y - point->y)) * kCost1);
}

int Astar::calcF(Point *point)
{
	return point->G + point->H;
}

//找出开放列表中的F值最小的节点
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
//isIgnoreCorner是标志位，用于传入是否考虑死角，false表示考虑死角，死角不能通过，true表示不考虑死角，死角能通过
Point *Astar::findPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner)
{
	//首先置入起点，拷贝开辟一个节点，内外隔离
	openList.push_back(new Point(startPoint.x, startPoint.y));
	while (!openList.empty())
	{
		auto curPoint = getLeastFpoint(); //找到F值最小的点
		openList.remove(curPoint); //从开启列表中删除
		closeList.push_back(curPoint); //放到关闭列表
		//1.找到当前周围八个格子中可以通过的格子
		auto surroundPoints = getSurrondPoints(curPoint, isIgnoreCorner);
		for (auto &target : surroundPoints)
		{
			//对于某个格子，如果它不在开启列表中，加入到开启列表，设置当前格子为其父节点，计算FGH
			if (!isInList(openList, target))
			{
				//target的父节点设置为curPoint
				target->parent = curPoint;
				target->G = calcG(curPoint, target);
				target->H = calcH(target, &endPoint);
				target->F = calcF(target);
				openList.push_back(target);
			}
			//对于一个格子，它在开启列表中，计算G值，如果比原来的大，就什么都不做，否则设置它的父节点为当前节点，并更新其G和F
			else
			{
				//tempG是经过当前节点的G值
				int tempG = calcG(curPoint, target);
				//经过当前节点的G值比原来的G值小，表示经过当前节点是较为优的路径，则要进行更新GHF
				if (tempG < target->G)
				{
					target->parent = curPoint;
					target->G = tempG;
					target->F = calcF(target);
				}
			}
			//判断终点endPoint是否在openList中
			Point *resPoint = isInList(openList, &endPoint);
		    //如果终点endPoint是在openList中，那么返回终点的深拷贝
			if (resPoint)
			{
				return resPoint; //返回列表里的节点指针，不要用原来传入的endpoint指针，因为发生了深拷贝
			}
		}
	}
	//openList为空而且resPoint不在其中，返回NULL
	return NULL;
}

//返回路径的算法
std::list<Point *> Astar::GetPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner)
{
	//得到最终节点
	Point *result = findPath(startPoint, endPoint, isIgnoreCorner);
	std::list<Point *> path;
	//返回路径，路径是以双线链表List的形式存在的
	while (result)
	{
		path.push_back(result);
		result = result->parent;
	}
	return path;
}

//判断point是不是在列表list中
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
	//如果点与当前节点重合，超出地图、是障碍物、或者在关闭列表中，返回false,表示当前节点不能用于下一步判断
	if (target->x<0 || target->x>maze.size() - 1 || target->y<0 || target->y>maze[0].size() - 1 || maze[target->x][target->y] == 1 || (target->x == point->x &&target->y == point->y) || isInList(closeList, target))
		return false;
	else
	{
		if (abs(point->x - target->x) + abs(point->y - target->y) == 1) //不是斜角
			return true;
		else
		{
			//斜对角要判断是否被绊住
			if (maze[point->x][target->y] == 0 && maze[target->x][point->y] == 0)
				return true;
			else
				return isIgnoreCorner; //如果设置的是忽略绊住的情况，依然有可能返回true
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