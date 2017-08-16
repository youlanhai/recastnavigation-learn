//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"

//两个AABB是否相交
inline bool overlapBounds(const float* amin, const float* amax, const float* bmin, const float* bmax)
{
	bool overlap = true;
	overlap = (amin[0] > bmax[0] || amax[0] < bmin[0]) ? false : overlap;
	overlap = (amin[1] > bmax[1] || amax[1] < bmin[1]) ? false : overlap;
	overlap = (amin[2] > bmax[2] || amax[2] < bmin[2]) ? false : overlap;
	return overlap;
}

//区间是否相交
inline bool overlapInterval(unsigned short amin, unsigned short amax,
							unsigned short bmin, unsigned short bmax)
{
	if (amax < bmin) return false;
	if (amin > bmax) return false;
	return true;
}


static rcSpan* allocSpan(rcHeightfield& hf)
{
	// If running out of memory, allocate new page and update the freelist.
	if (!hf.freelist || !hf.freelist->next)
	{
		// Create new page.
		// Allocate memory for the new pool.
		rcSpanPool* pool = (rcSpanPool*)rcAlloc(sizeof(rcSpanPool), RC_ALLOC_PERM);
		if (!pool) return 0;
		pool->next = 0;
		// Add the pool into the list of pools.
		pool->next = hf.pools;
		hf.pools = pool;
		// Add new items to the free list.
		rcSpan* freelist = hf.freelist;
		rcSpan* head = &pool->items[0];
		rcSpan* it = &pool->items[RC_SPANS_PER_POOL];
		do
		{
			--it;
			it->next = freelist;
			freelist = it;
		}
		while (it != head);
		hf.freelist = it;
	}
	
	// Pop item from in front of the free list.
	rcSpan* it = hf.freelist;
	hf.freelist = hf.freelist->next;
	return it;
}

static void freeSpan(rcHeightfield& hf, rcSpan* ptr)
{
	if (!ptr) return;
	// Add the node in front of the free list.
	ptr->next = hf.freelist;
	hf.freelist = ptr;
}

/** 添加一个区间
 @param hf      高度场数据结构
 @param x       格子坐标x
 @param y       格子坐标z
 @param smin    y方向上的最小值
 @param smax    y方向上的最大值
 @param area    是否可通行
 @param flagMergeThr 爬行高度
 */
static void addSpan(rcHeightfield& hf, const int x, const int y,
					const unsigned short smin, const unsigned short smax,
					const unsigned char area, const int flagMergeThr)
{
	// 得到二维数组索引
	int idx = x + y*hf.width;
	
    // 分配区间数据结构
	rcSpan* s = allocSpan(hf);
	s->smin = smin;
	s->smax = smax;
	s->area = area;
	s->next = 0;
	
    //这个单元是空的，直接添加
	// Empty cell, add he first span.
	if (!hf.spans[idx])
	{
		hf.spans[idx] = s;
		return;
	}

	rcSpan* prev = 0;
	rcSpan* cur = hf.spans[idx];
	
	// Insert and merge spans.
	while (cur)
	{
		if (cur->smin > s->smax)//s在cur之下
		{
			// Current span is further than the new span, break.
			break;
		}
		else if (cur->smax < s->smin)//s在cur之上
		{
			// Current span is before the new span advance.
			prev = cur;
			cur = cur->next;
		}
		else//介于两者之间
		{
            //合并两个spans，将s的范围进行扩充，以包含cur
			// Merge spans.
			if (cur->smin < s->smin)
				s->smin = cur->smin;
			if (cur->smax > s->smax)
				s->smax = cur->smax;
			
            //合并域参数
			// Merge flags.
			if (rcAbs((int)s->smax - (int)cur->smax) <= flagMergeThr)
				s->area = rcMax(s->area, cur->area);
			
            //将cur span删除，然后重新归并span
			// Remove current span.
			rcSpan* next = cur->next;
			freeSpan(hf, cur);
			if (prev)
				prev->next = next;
			else
				hf.spans[idx] = next;
			cur = next;
		}
	}
	
    //插入新的span
	// Insert new span.
	if (prev)
	{
		s->next = prev->next;
		prev->next = s;
	}
	else
	{
		s->next = hf.spans[idx];
		hf.spans[idx] = s;
	}
}

/// @par
///
/// The span addition can be set to favor flags. If the span is merged to
/// another span and the new @p smax is within @p flagMergeThr units
/// from the existing span, the span flags are merged.
///
/// @see rcHeightfield, rcSpan.
void rcAddSpan(rcContext* /*ctx*/, rcHeightfield& hf, const int x, const int y,
			   const unsigned short smin, const unsigned short smax,
			   const unsigned char area, const int flagMergeThr)
{
//	rcAssert(ctx);
	addSpan(hf, x,y, smin, smax, area, flagMergeThr);
}

/** 使用平面切割多边形。这里只沿着垂直方向(x=d or z = d)切，不管水平方向。所以法线y一直是0，就不用传参了。
 @param in      输入多边形的顶点数据
 @param n       输入顶点的数量
 @param pnx     平面法线的x值
 @param pnz     平面法线的z值
 @param pd      平面距离的负值
 @return 返回在平面正向上的点的个数。
 
 平面方程：p = n * d
 
 */
static int clipPoly(const float* in, int n, float* out, float pnx, float pnz, float pd)
{
	float d[12];//点到平面的距离。12表示最多12边形
    
    // 点乘求得顶点到平面的距离 d[i] = dot(vertex[i], pNormal) - pd
    // pd外面传了负值，所以下面的公式就变成了加上pd
	for (int i = 0; i < n; ++i)
		d[i] = pnx*in[i*3+0] + pnz*in[i*3+2] + pd;
	
	int m = 0;
    // i表示当前顶点的索引，j表示上一个顶点的索引
	for (int i = 0, j = n-1; i < n; j=i, ++i)
	{
		bool ina = d[j] >= 0; //起点
		bool inb = d[i] >= 0; //终点
		if (ina != inb)//前后两点不在同一侧，插值计算交点，将交点记录下来。
		{
			float s = d[j] / (d[j] - d[i]);
            // out[m] = in[j] * (1.0 - s) + in[i] * s;
			out[m*3+0] = in[j*3+0] + (in[i*3+0] - in[j*3+0])*s;
			out[m*3+1] = in[j*3+1] + (in[i*3+1] - in[j*3+1])*s;
			out[m*3+2] = in[j*3+2] + (in[i*3+2] - in[j*3+2])*s;
			m++;
		}
		if (inb)//终点在正面，则记下该点。
		{
			out[m*3+0] = in[i*3+0];
			out[m*3+1] = in[i*3+1];
			out[m*3+2] = in[i*3+2];
			m++;
		}
	}
	return m;
}

/** 栅格化一个三角形
 @param v0      顶点1
 @param v1      顶点2
 @param v2      顶点3
 @param area    是否可通过。
 @param hf      高度域（体素数据结构）
 @param bmin    世界包围盒最小点(xyz)
 @param bmax    世界包围盒最大点(xyz)
 @param cs      cell size，单元格水平(xz)尺寸。ch表示单元格的高度值(cell height)。
 @param ics     1 / cs。用于快速计算除法 n = x / cs
 @param ich     1 / ch。用于快速计算除法 n = y / ch
 @param flagMergeThr 最大爬行高度
 */
static void rasterizeTri(const float* v0, const float* v1, const float* v2,
						 const unsigned char area, rcHeightfield& hf,
						 const float* bmin, const float* bmax,
						 const float cs, const float ics, const float ich,
						 const int flagMergeThr)
{
	const int w = hf.width;
	const int h = hf.height;
	float tmin[3], tmax[3];
	const float by = bmax[1] - bmin[1];
	
    //计算三角形的包围盒
	// Calculate the bounding box of the triangle.
	rcVcopy(tmin, v0);
	rcVcopy(tmax, v0);
	rcVmin(tmin, v1);
	rcVmin(tmin, v2);
	rcVmax(tmax, v1);
	rcVmax(tmax, v2);
	
    //如果三角形的包围盒与heighfield不相交，则跳过这个三角形。
	// If the triangle does not touch the bbox of the heightfield, skip the triagle.
	if (!overlapBounds(bmin, bmax, tmin, tmax))
		return;
	
    //计算三角形包围盒，在x、z方向上与grid包围盒的比例。也就是求得包围盒所占据单元格的范围x=[x0, x1], z=[y0, y1]。
	// Calculate the footpring of the triangle on the grid.
	int x0 = (int)((tmin[0] - bmin[0])*ics);
	int y0 = (int)((tmin[2] - bmin[2])*ics);
	int x1 = (int)((tmax[0] - bmin[0])*ics);
	int y1 = (int)((tmax[2] - bmin[2])*ics);
    // 如果坐标超过范围，则截取到边界。
	x0 = rcClamp(x0, 0, w-1);
	y0 = rcClamp(y0, 0, h-1);
	x1 = rcClamp(x1, 0, w-1);
	y1 = rcClamp(y1, 0, h-1);
	
    //将三角形裁减到所有相交的grid中
	// Clip the triangle into all grid cells it touches.
	float in[7*3], out[7*3], inrow[7*3];
	
	for (int y = y0; y <= y1; ++y)
	{
		// Clip polygon to row.
		rcVcopy(&in[0], v0);
		rcVcopy(&in[1*3], v1);
		rcVcopy(&in[2*3], v2);
        
		int nvrow = 3; // 裁剪后的顶点数量. number vertices in row
        
        //将三角形裁减到z=[cz, cz+cs]平面之间。
        
        // 格子y坐标(z值)转换到世界坐标
		const float cz = bmin[2] + y*cs;
        
        // 先用z=1方向的面裁剪，得到前面的z+。平面距离传负值，方便里面计算。
		nvrow = clipPoly(in, nvrow, out, 0, 1, -cz);
		if (nvrow < 3) continue;
        // 再用z=-1方向的面裁剪，得到后面的z-。
		nvrow = clipPoly(out, nvrow, inrow, 0, -1, cz+cs);
		if (nvrow < 3) continue;
		
		for (int x = x0; x <= x1; ++x)
		{
            // 将三角形裁减到x=[cx, cx+cs]平面之间。
            
			// Clip polygon to column.
			int nv = nvrow;
			const float cx = bmin[0] + x*cs;
            // 裁剪得到前面
			nv = clipPoly(inrow, nv, out, 1, 0, -cx);
			if (nv < 3) continue;
            // 裁剪得到后面
			nv = clipPoly(out, nv, in, -1, 0, cx+cs);
			if (nv < 3) continue;
			
            //此时，in中存贮的为裁减后的多边形。

            //计算新多边形y轴的最大和最小值
			// Calculate min and max of the span.
			float smin = in[1], smax = in[1];//y坐标
			for (int i = 1; i < nv; ++i)
			{
				smin = rcMin(smin, in[i*3+1]);
				smax = rcMax(smax, in[i*3+1]);
			}
			smin -= bmin[1];
			smax -= bmin[1];

            //判断新多变形，是否与世界包围盒相交
			// Skip the span if it is outside the heightfield bbox
			if (smax < 0.0f) continue;//整个多边形位于包围盒之下，即不相交
			if (smin > by) continue;
			// Clamp the span to the heightfield bbox.
			if (smin < 0.0f) smin = 0;
			if (smax > by) smax = by;
			
            //得到y方向上得到倍数
			// Snap the span to the heightfield height grid.
			unsigned short ismin = (unsigned short)rcClamp((int)floorf(smin * ich), 0, RC_SPAN_MAX_HEIGHT);
			unsigned short ismax = (unsigned short)rcClamp((int)ceilf(smax * ich), (int)ismin+1, RC_SPAN_MAX_HEIGHT);
			
			addSpan(hf, x, y, ismin, ismax, area, flagMergeThr);
		}
	}
}

/// @par
///
/// No spans will be added if the triangle does not overlap the heightfield grid.
///
/// @see rcHeightfield
void rcRasterizeTriangle(rcContext* ctx, const float* v0, const float* v1, const float* v2,
						 const unsigned char area, rcHeightfield& solid,
						 const int flagMergeThr)
{
	rcAssert(ctx);

	ctx->startTimer(RC_TIMER_RASTERIZE_TRIANGLES);

	const float ics = 1.0f/solid.cs;
	const float ich = 1.0f/solid.ch;
	rasterizeTri(v0, v1, v2, area, solid, solid.bmin, solid.bmax, solid.cs, ics, ich, flagMergeThr);

	ctx->stopTimer(RC_TIMER_RASTERIZE_TRIANGLES);
}

/// 栅格化三角形列表。转换成体素，或者叫Span
/// @par
///
/// Spans will only be added for triangles that overlap the heightfield grid.
///
/// @see rcHeightfield
void rcRasterizeTriangles(rcContext* ctx, const float* verts, const int /*nv*/,
						  const int* tris, const unsigned char* areas, const int nt,
						  rcHeightfield& solid, const int flagMergeThr)
{
	rcAssert(ctx);

	ctx->startTimer(RC_TIMER_RASTERIZE_TRIANGLES);
	
	const float ics = 1.0f/solid.cs;
	const float ich = 1.0f/solid.ch;
	// Rasterize triangles.
	for (int i = 0; i < nt; ++i)
	{
		const float* v0 = &verts[tris[i*3+0]*3];
		const float* v1 = &verts[tris[i*3+1]*3];
		const float* v2 = &verts[tris[i*3+2]*3];
		// Rasterize.
		rasterizeTri(v0, v1, v2, areas[i], solid, solid.bmin, solid.bmax, solid.cs, ics, ich, flagMergeThr);
	}
	
	ctx->stopTimer(RC_TIMER_RASTERIZE_TRIANGLES);
}

/// @par
///
/// Spans will only be added for triangles that overlap the heightfield grid.
///
/// @see rcHeightfield
void rcRasterizeTriangles(rcContext* ctx, const float* verts, const int /*nv*/,
						  const unsigned short* tris, const unsigned char* areas, const int nt,
						  rcHeightfield& solid, const int flagMergeThr)
{
	rcAssert(ctx);

	ctx->startTimer(RC_TIMER_RASTERIZE_TRIANGLES);
	
	const float ics = 1.0f/solid.cs;
	const float ich = 1.0f/solid.ch;
	// Rasterize triangles.
	for (int i = 0; i < nt; ++i)
	{
		const float* v0 = &verts[tris[i*3+0]*3];
		const float* v1 = &verts[tris[i*3+1]*3];
		const float* v2 = &verts[tris[i*3+2]*3];
		// Rasterize.
		rasterizeTri(v0, v1, v2, areas[i], solid, solid.bmin, solid.bmax, solid.cs, ics, ich, flagMergeThr);
	}
	
	ctx->stopTimer(RC_TIMER_RASTERIZE_TRIANGLES);
}

/// @par
///
/// Spans will only be added for triangles that overlap the heightfield grid.
///
/// @see rcHeightfield
void rcRasterizeTriangles(rcContext* ctx, const float* verts, const unsigned char* areas, const int nt,
						  rcHeightfield& solid, const int flagMergeThr)
{
	rcAssert(ctx);
	
	ctx->startTimer(RC_TIMER_RASTERIZE_TRIANGLES);
	
	const float ics = 1.0f/solid.cs;
	const float ich = 1.0f/solid.ch;
	// Rasterize triangles.
	for (int i = 0; i < nt; ++i)
	{
		const float* v0 = &verts[(i*3+0)*3];//一个顶点占3个float，故*3
		const float* v1 = &verts[(i*3+1)*3];
		const float* v2 = &verts[(i*3+2)*3];
		// Rasterize.
		rasterizeTri(v0, v1, v2, areas[i], solid, solid.bmin, solid.bmax, solid.cs, ics, ich, flagMergeThr);
	}
	
	ctx->stopTimer(RC_TIMER_RASTERIZE_TRIANGLES);
}
