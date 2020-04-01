#include <stdio.h>
#include <stdlib.h>

#define START	        1
#define END  	        2
#define OBSTRUCTS		3

typedef struct Point
{
	int xx;	// 坐标(x,y)
	int yy;
	int gg;	// 起点到此点的距离G
	int	hh;	// 启发函数预测的此点到终点的距离H
	int style;// 结点类型：起始点1，终点2，障碍物3
	struct Point * parent;	// 父节点
	int is_in_closetable;		// 是否在close表中
	int is_in_opentable;		// 是否在open表中
}Point, *APoint;

Point  map_maze[10][10];		// 结点数组，地图
APoint open_table[100];		// open表
APoint close_table[100];		// close表
APoint path_stack[100];		// 保存路径
int	   open_node_count;		// open表中节点数量
int	   close_node_count;		// close表中结点数量
int        top = -1;			// 栈顶


//堆排序找最小,用于open表排序
void swap( int idx1, int idx2 )  
{  
	APoint temp = open_table[idx1];
	open_table[idx1] = open_table[idx2];
	open_table[idx2] = temp;
}  
void adjust( int /*i*/nIndex )    
{ 	
	int cur = nIndex;
	int child = cur * 2 + 1;	
	int parent = ( cur - 1 ) / 2;	
 
	if (nIndex < 0 || nIndex >= open_node_count)
	{
		return;
	}
	
	
	while ( child < open_node_count )
	{
		
		if ( child + 1 < open_node_count && open_table[child]->gg + open_table[child]->hh  > open_table[child+1]->gg + open_table[child+1]->hh )
		{
			++child;				
		}
		
		if (open_table[cur]->gg + open_table[cur]->hh <= open_table[child]->gg + open_table[child]->hh)
		{
			break;                  
		}
		else
		{
			swap( child, cur );			
			cur = child;				
			child = cur* 2 + 1;			
		}
	}
	
	if (cur != nIndex)
	{
		return;
	}
 
	while (cur != 0)
	{
		if (open_table[cur]->gg + open_table[cur]->hh >= open_table[parent]->gg + open_table[parent]->hh)
		{
			break;
		}
		else
		{
			swap( cur, parent );
			cur = parent;
			parent = (cur-1)/2;
		}
	}
}  



// 判断邻居点是否可以进入open表
void if_insert_to_opentable( int x, int y, APoint cur_node, APoint end_node, int w )
{
	int i;
 
	if ( map_maze[x][y].style != OBSTRUCTS )		// 不是障碍物
	{
		if ( !map_maze[x][y].is_in_closetable )	// 不在close表中
		{
			if ( map_maze[x][y].is_in_opentable )	// 在open表中
			{
				// 是否是更优化的路径
				
				if ( map_maze[x][y].gg > cur_node->gg + w )	// 如果更优化
				{
					map_maze[x][y].gg = cur_node->gg + w;
					map_maze[x][y].parent = cur_node;
 
					for ( i = 0; i < open_node_count; ++i )
					{
						if ( open_table[i]->xx == map_maze[x][y].xx && open_table[i]->yy == map_maze[x][y].yy )
						{
							break;
						}
					}
 
					adjust( i );					// 下面调整点
				}
			}
			else									// 不在open表中
			{
				map_maze[x][y].gg = cur_node->gg + w;
				map_maze[x][y].hh = abs(end_node->xx - x ) + abs(end_node->yy - y);
				map_maze[x][y].parent = cur_node;
				map_maze[x][y].is_in_opentable = 1;
				open_table[open_node_count++] = &(map_maze[x][y]);
			}
		}
	}
}
 
// 查找邻居点，对周围共8个邻居点进行查找
 
void get_neighbors( APoint cur_node, APoint end_node )
{
	int x = cur_node->xx;
	int y = cur_node->yy;
 
	// 下面对于8个邻居进行处理！
	// 
	if ( ( x + 1 ) >= 0 && ( x + 1 ) < 10 && y >= 0 && y < 10 )
	{
		if_insert_to_opentable( x+1, y, cur_node, end_node, 10 );
	}
 
	if ( ( x - 1 ) >= 0 && ( x - 1 ) < 10 && y >= 0 && y < 10 )
	{
		if_insert_to_opentable( x-1, y, cur_node, end_node, 10 );
	}
 
	if ( x >= 0 && x < 10 && ( y + 1 ) >= 0 && ( y + 1 ) < 10 )
	{
		if_insert_to_opentable( x, y+1, cur_node, end_node, 10 );
	}
 
	if ( x >= 0 && x < 10 && ( y - 1 ) >= 0 && ( y - 1 ) < 10 )
	{
		if_insert_to_opentable( x, y-1, cur_node, end_node, 10 );
	}
 
	if ( ( x + 1 ) >= 0 && ( x + 1 ) < 10 && ( y + 1 ) >= 0 && ( y + 1 ) < 10 )
	{
		if_insert_to_opentable( x+1, y+1, cur_node, end_node, 14 );
	}
 
	if ( ( x + 1 ) >= 0 && ( x + 1 ) < 10 && ( y - 1 ) >= 0 && ( y - 1 ) < 10 )
	{
		if_insert_to_opentable( x+1, y-1, cur_node, end_node, 14 );
	}
 
	if ( ( x - 1 ) >= 0 && ( x - 1 ) < 10 && ( y + 1 ) >= 0 && ( y + 1 ) < 10 )
	{
		if_insert_to_opentable( x-1, y+1, cur_node, end_node, 14 );
	}
 
	if ( ( x - 1 ) >= 0 && ( x - 1 ) < 10 && ( y - 1 ) >= 0 && ( y - 1 ) < 10 )
	{
		if_insert_to_opentable( x-1, y-1, cur_node, end_node, 14 );
	}
}

int main()
{ 
	// 地图数组的定义
	// 
	Point *start;			// 起始点
	Point *end;			// 结束点
	Point *current;			// 当前点
	int       is_found;			// 是否找到路径
	int maze[][10] ={			// 自定义
						{ 1,0,3,3,0,3,0,0,0,0 },
						{ 0,0,3,0,0,3,0,3,0,0 },
						{ 0,0,3,0,0,3,3,3,0,0 },
						{ 0,0,3,0,0,0,0,0,0,3 },
						{ 3,0,0,0,0,3,0,0,0,0 },
						{ 3,3,0,3,0,0,3,3,0,3 },
						{ 3,0,0,0,0,3,3,0,0,0 },
						{ 0,3,0,0,0,0,0,0,0,0 },
						{ 3,3,3,0,0,3,0,3,2,3 },
						{ 3,0,0,0,0,3,3,3,0,3 },
					};
	int		  i,j,x;
	
	// 准备map_maze
	for( i = 0; i < 10; ++i )
	{
		for ( j = 0; j < 10; ++j )
		{
			map_maze[i][j].gg = 0;
			map_maze[i][j].hh = 0;
			map_maze[i][j].is_in_closetable = 0;
			map_maze[i][j].is_in_opentable = 0;
			map_maze[i][j].style = maze[i][j];
			map_maze[i][j].xx = i;
			map_maze[i][j].yy = j;
			map_maze[i][j].parent = NULL;
 
			if ( map_maze[i][j].style == START )	// 起点
			{
				start = &(map_maze[i][j]);
			}
			else if( map_maze[i][j].style == END )	// 终点
			{
				end = &(map_maze[i][j]);
			}
 
			printf("%d ", maze[i][j]);
		}
 
		printf("\n");
	}
 
	// 使用A*算法得到路径
	
	open_table[open_node_count++] = start;			// 起始点加入open表
	
	start->is_in_opentable = 1;				// 加入open表
	start->gg = 0;
	start->hh = abs(end->xx - start->xx) + abs(end->yy - start->yy);
	start->parent = NULL;
	
	if ( start->xx == end->xx && start->yy == end->yy )
	{
		printf("起点==终点！\n");
		return 0;
	}
	
	is_found = 0;
 
	while( 1 )
	{
		
		current = open_table[0];		// open表的第一个点是f值最小的点
		open_table[0] = open_table[--open_node_count];	// 最后一个点放到第一个点
		adjust( 0 );				// 堆排序
		
		close_table[close_node_count++] = current;	// 当前点加入close表
		current->is_in_closetable = 1;		
 
		if ( current->xx == end->xx && current->yy == end->yy )// 终点在close表中，搜索结束
		{
			is_found = 1;
			break;
		}
 
		get_neighbors( current, end );			// 处理邻居点
 
		if ( open_node_count == 0 )				// 没有路径到达
		{
			is_found = 0;
			break;
		}
	}
 
	if ( is_found )
	{
		current = end;
		
		while( current )
		{
			path_stack[++top] = current;
			current = current->parent;
		}
 
		while( top >= 0 )		//输出路径
		{
			if ( top > 0 )
			{
				printf("(%d,%d)-->", path_stack[top]->xx, path_stack[top--]->yy);
			}
			else
			{
				printf("(%d,%d)", path_stack[top]->xx, path_stack[top--]->yy);
			}
		}
	}
	else
	{
		printf("Did not find path.Sorry!");
	}
 
	puts("");
 
	return 0;
}