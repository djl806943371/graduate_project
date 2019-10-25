#ifndef CUSTOMBUTTON_H
#define CUSTOMBUTTON_H

#include <QtWidgets/QWidget>

#include <QPoint>
#include "qmath.h"

enum QUADRANT_NUM {
    QUADRANT_UP,
    QUADRANT_LEFT,
    QUADRANT_DOWN,
    QUADRANT_RIGHT,
    QUADRANT_NO_DEFINE,
};

class CustomButton : public QWidget
{
    Q_OBJECT

public:
    CustomButton(QWidget* parent = nullptr);
    // 设置弧长及半径;
    void setRadiusValue(int radius);
    void setArcLength(int arcLength);
    void drawRotatedText(QPainter *painter, float degrees, int x, int y, const QString &text);

    QPixmap getPixmap(const int ping);
    QColor getColor(const int ping);
    int getLineNum(const int ping);
    QPixmap getSignalPixmap(const QColor &color,const int lineNum);//获取信号位图

    void setBeginDegree(int degree);



    void setStrUp(QString str){mStrUp = str;}
    void setStrLeft(QString str){mStrLeft = str;}
    void setStrDown(QString str){mStrDown = str;}
    void setStrRight(QString str){mStrRight = str;}
    void setSPLcolor(QColor color){colorSPL = color;update();}
    void setSectorColor(QColor color){mSectorColor = color;update();}
    void setBKGcolor(QColor color){colorBKG = color;update();}
    void setWidgetStyle(QString style);

    void setAxesVertical(bool axesVertical);

private:
    // 初始化按钮;
    void initButton();
    // 绘制按钮;
    void paintEvent(QPaintEvent *);
    // 添加圆弧;
    void addArc(int x, int y, int startAngle, int angleLength, QColor color);

    // 鼠标事件;
    void mouseMoveEvent(QMouseEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);

    int analysisAngle(int x,int y);

signals:
    // 鼠标点击;
    void signalButtonClicked();
    // 鼠标松开;
    void signalButtonReleased();
    // 按钮拖动;
    void signalButtonMoved(int degree, int angle);
private:
    // 弧长及半径;
    int m_radius, m_arcLength;

    // 圆弧路径;
    QList<QPainterPath> m_arcPathList;
    QList<QPainterPath> m_textPathList;
    // 圆弧颜色;
    QList<QBrush> m_colorList;
    // 当前鼠标按钮/进入 按钮的索引;
    int m_pressIndex, m_enterIndex;
    // 鼠标事件标志位;
    bool m_isMousePressed;
    bool m_isMouseEntered;

    int mCurAngle;

    QPoint mCenterRound;
    QPixmap mDegreePixmap;

    QString mStrUp;
    QString mStrLeft;
    QString mStrDown;
    QString mStrRight;
    QColor mSectorColor;


    //beijing分割线颜色
    QColor colorBKG = QColor(41, 44, 50);
    QColor colorSPL = QColor(32, 149, 216);
    QColor colorSectorUp2 = QColor(68, 68, 68);
    QColor colorSectorUp = QColor(60, 60, 60);
    QColor colorSectorDown = QColor(22, 22, 22);

    QColor colorbgGradient0 = QColor(24, 24, 24);
    QColor colorbgGradient1 = QColor(53, 57, 63);

    QColor colorExcircle0 = QColor(68, 68, 68);
    QColor colorExcircle5 = QColor(37, 40, 46);
    QColor colorExcircle9 = QColor(22, 22, 22);

    QColor colorInnerCircle0 = QColor(38, 40, 46);
    QColor colorInnerCircle9 = QColor(45, 48, 55);

    bool mAxesVertical = false;

};

#endif
