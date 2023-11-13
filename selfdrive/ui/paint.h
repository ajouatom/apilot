#pragma once

#include "selfdrive/ui/ui.h"

void ui_draw(UIState *s, int w, int h);
void ui_update_alert(const Alert& a);
void ui_draw_alert(UIState* s);
void ui_draw_image(const UIState *s, const Rect &r, const char *name, float alpha);
void ui_draw_rect(NVGcontext *vg, const Rect &r, NVGcolor color, int width, float radius = 0);
void ui_fill_rect(NVGcontext *vg, const Rect &r, const NVGpaint &paint, float radius = 0);
void ui_fill_rect(NVGcontext *vg, const Rect &r, const NVGcolor &color, float radius = 0);
void ui_nvg_init(UIState *s);
void ui_resize(UIState *s, int width, int height);
void ui_update_params(UIState *s);


#define PLOT_MAX 500
class DrawPlot : public QObject{
	Q_OBJECT
protected:
	int     plotSize = 0;
	int     plotIndex = 0;
	float   plotQueue[2][PLOT_MAX];
	float   plotMin = 0.;
	float   plotMax = 0.;
	float   plotShift = 0.0;
	float   plotX = 300.0;
	float   plotWidth = 1000;
	float   plotY = 30.0;
	float   plotHeight = 300.0;
	float   plotRatio = 1.0;
	int     show_plot_mode_prev = -1;
	void	drawPlotting(const UIState* s, int start, float x, float y[], int size, NVGcolor* color, float stroke = 0.0);
	void	makePlotData(const UIState* s, float& data1, float& data2);

public:
	void	draw(const UIState* s);
};

class DrawApilot : public QObject {
	Q_OBJECT

protected:
	float	m_fLeadDistRadar;
	float	m_fLeadDistVision;
	float	m_fStopDist;
	bool	m_bLeadStatus;
	float	getRadarDist() { return m_fLeadDistRadar; }
	float	getVisionDist() { return m_fLeadDistVision; }
	float	getStopDist() { return m_fStopDist; }
	bool	isRadarDetected() { return m_fLeadDistRadar > 0.0; }
	bool	isLeadDetected() { return m_bLeadStatus; }
	void	makeLeadData(const UIState* s);

	void	drawBackground(const UIState* s);
	void	drawRadarInfo(const UIState* s);
	void	drawAccel(const UIState* s, int x, int y);
	void	drawRpm(const UIState* s, int x, int y);
	void	drawGapInfo(const UIState* s, int x, int y);
	void	drawConnInfo(const UIState* s);
	void	drawSpeed(const UIState* s, int x, int y);
	void	drawTurnInfo(const UIState* s, int x, int y);
	void	drawSteer(const UIState* s, int x, int y);
	void	drawPathEnd(const UIState* s, int x, int y, int path_x, int path_y, int path_width);

	void	makePathXY(const UIState* s, int& path_bx, int& path_x, int& path_y, int& path_width);
	void	makeData(const UIState* s);

	DrawPlot	drawPlot;
	int		m_blinkerTimer = 0;
	int		m_longActiveUser = 0;
	int		m_longActiveUserReady = 0;
	bool	isLongActive() { return m_longActiveUser > 0; }
	bool	isLongActiveReady() { return m_longActiveUserReady > 0; }
	bool	m_brakeHoldActive = false;
	bool	m_softHoldActive = false;
	bool	isBrakeHold() { return m_brakeHoldActive; }
	bool	isSoftHold() { return m_softHoldActive; }
	bool	m_enabled = false;
	bool	isEnabled() { return m_enabled; }
	bool	isBlinkerOn() { return m_blinkerTimer <= (16 / 2); }
	float	m_vEgo = 0.0;
	float	m_dispSpeed = 0.0;
	float	getVEgo() { return m_vEgo; }
	int		m_trafficMode = 0;
	int		getTrafficMode() { return m_trafficMode; }

	void	drawTPMS(const UIState* s);
	void	drawDateTime(const UIState* s);


	QString m_navText = "";
	int icon_size = 256;

public:
	DrawApilot();
public:
	void drawLaneLines(const UIState* s);
	void drawLeadApilot(const UIState* s);
	void drawDebugText(UIState* s, bool show);
	void drawDeviceState(UIState* s, bool show);


	


};
