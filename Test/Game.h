//Game.h
#pragma once

#include "../../LZ3DEngine/LZ3DEngine.h"
#include "../../LZData/lzdheader.h"

//#define USE_GUI

#ifdef USE_GUI
#include "../../LZGUI/LZGUI.h"
#endif


//游戏类
class CGame :public CApp
{
public:
	CGame();
	~CGame();

    /*注册窗口类消息。请实现该消息来修改窗口风格。*/
    void onRegisterClass(WNDCLASSEX *pwc);

    /*创建设备消息。实现该方法来修改设备。*/
    void onCreateDevice(D3DPRESENT_PARAMETERS * pParam);

    /*游戏初始化*/
    bool init(void);

    /*游戏更新*/
    void update(void);

    /*渲染*/
    void render(void);

    /*释放资源*/
    void clear(void);

	void updateCamera(float fElapse);
	
    /**窗口过程。不需要调用默认窗口处理过程，如果未处理消息请返回0，否则请返回非0。*/
    virtual LRESULT wndProc(HWND ,UINT ,WPARAM ,LPARAM );
public:
	/*注意：所有指针成员，需在构造函数中进行赋空值初始化。*/
	bool		        m_bGameStart;
	RefPtr<CCursorCamera>	m_pCamera;
	RefPtr<cFpsRender>	    m_pFPS;
	RefPtr<CCube>		m_pCube;
    RefPtr<CSquare>     m_pSquare;

#ifdef USE_GUI
    RefPtr<CGUIManager> m_pGUIMgr;
#endif
};