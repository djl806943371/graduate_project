#include "CustomButton.h"
#include <QPainter>
#include <QMouseEvent>
#include <QDebug>
#include <QPushButton>
#include <QToolTip>
#include <qmath.h>

CustomButton::CustomButton(QWidget* parent)
    : QWidget(parent)
    , m_pressIndex(0)
    , m_enterIndex(0)
    , m_isMouseEntered(false)
    , m_isMousePressed(false)
    , m_radius(80)
    , m_arcLength(55)
    , mCenterRound(QPoint(0,0))
{
    m_bTextModeEn = false;
    this->setMouseTracking(true);

    mSectorColor = QColor(38,38,38);

    initButton();
    setWidgetStyle("Dark");
    setAxesVertical(false);
}

void CustomButton::setWidgetStyle(QString style)
{
    if(style == "Bright"){
        mSectorColor = QColor(238,241,240);
        colorSPL = QColor(63,155,178);
        colorBKG = QColor(193, 199, 209);

        colorSectorUp2 = QColor(240, 243, 208);
        colorSectorUp = QColor(239, 242, 247);
        colorSectorDown = QColor(221, 225, 231);

        colorbgGradient0 = QColor(175, 180, 191);
        colorbgGradient1 = QColor(239, 242, 247);

        colorExcircle0 = QColor(211,215,223);
        colorExcircle5 = QColor(231,235,240);
        colorExcircle9 = QColor(182,187,197);
        colorInnerCircle0 = QColor(45, 48, 56);
        colorInnerCircle9 = QColor(30, 32, 37);
    }else{
        mSectorColor = QColor(38,38,38);
        colorSPL = QColor(32, 149, 216);
        colorBKG = QColor(41, 44, 50);

        colorSectorUp2 = QColor(68, 68, 68);
        colorSectorUp = QColor(60, 60, 60);
        colorSectorDown = QColor(22, 22, 22);

        colorbgGradient0 = QColor(24, 24, 24);
        colorbgGradient1 = QColor(53, 57, 63);

        colorExcircle0 = QColor(68, 68, 68);
        colorExcircle5 = QColor(37, 40, 46);
        colorExcircle9 = QColor(22, 22, 22);
        colorInnerCircle0 = QColor(45, 48, 56);
        colorInnerCircle9 = QColor(30, 32, 37);
    }
    update();
}

void CustomButton::setRadiusValue(int radius)
{
    m_radius = radius;
}

void CustomButton::setArcLength(int arcLength)
{
    m_arcLength = arcLength;
}

void CustomButton::drawRotatedText(QPainter *painter, float degrees, int x, int y, const QString &text)
{
    painter->save(); //保存原来坐标系统
    painter->translate(x, y); //平移坐标原点到 x， y
    painter->rotate(degrees); //坐标旋转degrees 度
    painter->drawText(0, 0, text); //在原点绘制文本
    painter->restore(); //回复原来的坐标系统
}

void CustomButton::setAxesVertical(bool axesVertical)
{
    mAxesVertical = axesVertical;
    if(mAxesVertical == false){
        addArc(1,0,45, 90, mSectorColor);
        addArc(0,1,135, 90, mSectorColor);
        addArc(-1,0,225, 90, mSectorColor);
        addArc(0,-1,315, 90, mSectorColor);
    }else{
        addArc(1,0,0, 90, mSectorColor);
        addArc(0,1,90, 90, mSectorColor);
        addArc(-1,0,180, 90, mSectorColor);
        addArc(0,-1,270, 90, mSectorColor);
    }
    // 绘制中心圆;
    QPainterPath centerRoundPath;
    centerRoundPath.addEllipse(QPoint(0, 0), m_radius - m_arcLength+2, m_radius - m_arcLength+2);
    m_arcPathList.append(centerRoundPath);
    m_colorList.append(QColor(255, 255, 255));


    // 添加文字;
    QFont font;
    font.setFamily("Microsoft YaHei");
    font.setPointSize(14);

    for (int i = 0; i < m_arcPathList.count(); i++)
    {
        QPainterPath painterPath;
        m_textPathList.append(painterPath);
    }

    mStrUp = QStringLiteral("小臂上仰");
    mStrLeft = QStringLiteral("手动收线");
    mStrDown = QStringLiteral("小臂下附");
    mStrRight = QStringLiteral("手动放线");
    update();
}

void CustomButton::initButton()
{
    addArc(1,0,45, 90, mSectorColor);
    addArc(0,1,135, 90, mSectorColor);
    addArc(-1,0,225, 90, mSectorColor);
    addArc(0,-1,315, 90, mSectorColor);

    // 绘制中心圆;
    QPainterPath centerRoundPath;
    centerRoundPath.addEllipse(QPoint(0, 0), m_radius - m_arcLength+2, m_radius - m_arcLength+2);
    m_arcPathList.append(centerRoundPath);
    m_colorList.append(QColor(255, 255, 255));


    // 添加文字;
    QFont font;
    font.setFamily("Microsoft YaHei");
    font.setPointSize(14);

    for (int i = 0; i < m_arcPathList.count(); i++)
    {
        QPainterPath painterPath;
        m_textPathList.append(painterPath);
    }

    mStrUp = QStringLiteral("小臂上仰");
    mStrLeft = QStringLiteral("手动收线");
    mStrDown = QStringLiteral("小臂下附");
    mStrRight = QStringLiteral("手动放线");
}

void CustomButton::paintEvent(QPaintEvent *)
{

    QPainter painter(this);
    painter.setRenderHint(QPainter::HighQualityAntialiasing, true);

    painter.setPen(Qt::NoPen);
    painter.translate(width()>>1, height()>>1);

    //背景色，分割线颜色
    painter.setBrush(colorBKG);
    painter.drawEllipse(QPoint(0,0), m_radius+8, m_radius+8);

    QLinearGradient linearGradient(0, -m_radius-2, 0, m_radius+2);
    linearGradient.setColorAt(0.0, colorSectorUp2);
    linearGradient.setColorAt(0.9, colorSectorDown);
    painter.setBrush(QBrush(linearGradient));
    painter.drawEllipse(QPoint(0, 0), m_radius+2, m_radius+2);

    linearGradient = QLinearGradient(0, -m_radius, 0, m_radius);
    linearGradient.setColorAt(0.0, colorSectorUp);
    linearGradient.setColorAt(0.9, colorSectorDown);
    painter.setBrush(QBrush(linearGradient));
    painter.drawEllipse(QPoint(0, 0), m_radius, m_radius);

//    linearGradient = QLinearGradient(0, -35, 0, 35);
//    linearGradient.setColorAt(0.5, QColor(24, 24, 24));
//    linearGradient.setColorAt(0.9, QColor(53, 57, 63));
//    painter.setBrush(QBrush(linearGradient));
//    painter.drawEllipse(QPoint(0, 0), m_radius - m_arcLength+4, m_radius - m_arcLength+4);

//    painter.setBrush(QColor(17,17,17));
//    painter.drawEllipse(QPoint(0, 0), m_radius - m_arcLength+2, m_radius - m_arcLength+2);

    QLinearGradient bgGradient(0, -37, 0, 37);
    bgGradient.setColorAt(0.0, colorbgGradient0);
    bgGradient.setColorAt(1.0, colorbgGradient1);
    painter.setBrush(bgGradient);
    painter.drawEllipse(QPoint(0, 0), m_radius - m_arcLength+4, m_radius - m_arcLength+4);

    //画分割线
    int count = 4;
    for(int i = 0; i < count;i++)
    {
        painter.save();
        if(mAxesVertical == false)
            painter.rotate(45+90 * i);
        else
            painter.rotate(0+90 * i);
        painter.setPen(QPen(colorBKG, 3, Qt::SolidLine));
        painter.drawLine(0, m_radius - m_arcLength+5, 0,m_radius+5);
        painter.setPen(QPen(colorSPL, 3, Qt::SolidLine, Qt::RoundCap));
        painter.drawLine(0, m_radius - m_arcLength+6, 0, 40);
        painter.setPen(QPen(colorSPL, 3, Qt::SolidLine));
        painter.drawLine(0, 40, 0, m_radius - 5);
        painter.restore();
    }


    linearGradient = QLinearGradient(0, mCenterRound.y() - m_radius + m_arcLength - 1, 0, mCenterRound.y()+m_radius - m_arcLength + 1);
    linearGradient.setColorAt(0.0, colorExcircle0);
    linearGradient.setColorAt(0.0, colorExcircle5);
    linearGradient.setColorAt(0.9, colorExcircle9);


    painter.setBrush(QBrush(linearGradient));
    painter.drawEllipse(mCenterRound, m_radius - m_arcLength , m_radius - m_arcLength );

//    linearGradient = QLinearGradient(0, mCenterRound.y() - m_radius + m_arcLength + 1, 0, mCenterRound.y() + m_radius - m_arcLength - 1);
//    linearGradient.setColorAt(0.0, colorInnerCircle0);
//    linearGradient.setColorAt(0.9, colorInnerCircle9);
//    painter.setBrush(QBrush(linearGradient));
//    painter.drawEllipse(mCenterRound, m_radius - m_arcLength -2, m_radius - m_arcLength -2);




    painter.setRenderHint(QPainter::SmoothPixmapTransform, true);
    if(mAxesVertical == false)
    {
        QMatrix leftmatrix;
        if(mCurWorkRegion == QUADRANT_UP){
            painter.drawPixmap(-30,-60,60,30,mDegreePixmap);
        }else if(mCurWorkRegion == QUADRANT_LEFT){
            leftmatrix.rotate(270);
            mDegreePixmap=mDegreePixmap.transformed(leftmatrix,Qt::SmoothTransformation);
            painter.drawPixmap(-60,-30,30,60,mDegreePixmap);
        }else if(mCurWorkRegion == QUADRANT_DOWN){
            leftmatrix.rotate(180);
            mDegreePixmap=mDegreePixmap.transformed(leftmatrix,Qt::SmoothTransformation);
            painter.drawPixmap(-30,30,60,30,mDegreePixmap);

        }else if(mCurWorkRegion == QUADRANT_RIGHT){
            leftmatrix.rotate(90);
            mDegreePixmap=mDegreePixmap.transformed(leftmatrix,Qt::SmoothTransformation);
            painter.drawPixmap(30,-30,30,60,mDegreePixmap);
        }
    }

    QFont font;
    font.setFamily("Microsoft YaHei");
    font.setPointSize(12);
    painter.setFont(font);
    painter.setPen(QColor(170,170,170));

    if(mAxesVertical == true){
        painter.drawText(QPoint(-60,-40),mStrUp);
        painter.drawText(QPoint(30,-40),mStrLeft);

        painter.drawText(QPoint(-80,40),mStrDown);
        painter.drawText(QPoint(20,40),mStrRight);
    }else{
        //m_bTextModeEn = true;
        QFontMetrics fm(font);
        int  rText = 65;

        int iTotalWidth;
        int nWidth;
        //行高空隙去除，-3或-4
        int nHeight = fm.height() - 4;

        //上方文字最简单
        iTotalWidth = fm.width(mStrUp);
        painter.save();
        painter.rotate(-90 * iTotalWidth / (rText * M_PI));
        for(int i = 0;i < mStrUp.size();i++)
        {
            //旋转一半，写文字，剩下一半再旋转
            nWidth = fm.width(mStrUp.at(i));
            painter.rotate(90 * nWidth / (rText * M_PI));
            painter.drawText(-nWidth / 2, -1 * rText, mStrUp.at(i));
            painter.rotate(90 * nWidth / (rText * M_PI));
        }
        painter.restore();

        //右侧文字，中文竖着，英文朝圆外

        painter.save();
        if (m_bTextModeEn)
        {
            iTotalWidth = fm.width(mStrRight);
            painter.rotate(90 - 90 * iTotalWidth / (rText * M_PI));
            for (int i = 0; i < mStrRight.size(); i++)
            {
                nWidth = fm.width(mStrRight.at(i));
                painter.rotate(90 * nWidth / (rText * M_PI));
                painter.drawText(-nWidth / 2, -1 * rText, mStrRight.at(i));
                painter.rotate(90 * nWidth / (rText * M_PI));
            }
        }
        else
        {
            iTotalWidth = nHeight*mStrRight.size();
            painter.rotate(-90 * iTotalWidth / (rText * M_PI));
            for (int i = 0; i < mStrRight.size(); i++)
            {

                painter.rotate(90 * nHeight / (rText * M_PI));
                //文字左右微调 -5
                painter.drawText(rText-5, nHeight / 2, mStrRight.at(i));
                painter.rotate(90 * nHeight / (rText * M_PI));
            }
        }
        painter.restore();

        //下方文字也简单，大家都一致
        painter.save();
        iTotalWidth = fm.width(mStrDown);
        painter.rotate(90 * iTotalWidth / (rText * M_PI));
        for (int i = 0; i < mStrDown.size(); i++)
        {
            nWidth = fm.width(mStrDown.at(i));
            painter.rotate(-90 * nWidth / (rText * M_PI));
            //文字上下微调 -8
            painter.drawText(-nWidth / 2, rText + nHeight-8, mStrDown.at(i));
            painter.rotate(-90 * nWidth / (rText * M_PI));
        }
        painter.restore();

        //左侧文字，中文朝上，英文朝内
        painter.save();
        if (m_bTextModeEn)
        {
            iTotalWidth = fm.width(mStrLeft);
            painter.rotate(90 + 90 * iTotalWidth / (rText * M_PI));
            for (int i = 0; i < mStrLeft.size(); i++)
            {
                nWidth = fm.width(mStrLeft.at(i));
                painter.rotate(-90 * nWidth / (rText * M_PI));
                painter.drawText(-nWidth / 2, rText+nHeight-8, mStrLeft.at(i));
                painter.rotate(-90 * nWidth / (rText * M_PI));
            }
        }
        else
        {
            iTotalWidth = nHeight*mStrLeft.size();
            painter.rotate(90 * iTotalWidth / (rText * M_PI));
            for (int i = 0; i < mStrLeft.size(); i++)
            {
                painter.rotate(-90 * nHeight / (rText * M_PI));
                //文字左右微调 -10
                painter.drawText(-rText-10, nHeight / 2, mStrLeft.at(i));
                painter.rotate(-90 * nHeight / (rText * M_PI));
            }
        }
        painter.restore();
    }



}

void CustomButton::addArc(int x, int y, int startAngle, int angleLength, QColor color)
{
    //绘制矩形 m_radius = 90

    //    QRectF(qreal left, qreal top, qreal width, qreal height);
    QRectF rect(-m_radius+x, -m_radius+y, m_radius * 2, m_radius * 2);

    // 设置扇形路径;
    QPainterPath path;
    path.arcTo(rect, startAngle, angleLength);
    QPainterPath subPath;
    // 设置小扇形路径;
    subPath.addEllipse(rect.adjusted(m_arcLength, m_arcLength, -m_arcLength, -m_arcLength));
    // 大扇形减去小扇形得到圆弧;
    path -= subPath;

    m_arcPathList.append(path);

    // 设置圆弧颜色
    QRadialGradient radialGradient;
    radialGradient.setCenter(0, 0);
    radialGradient.setRadius(m_radius);
    radialGradient.setColorAt(0, color);
    radialGradient.setColorAt(1.0, color);
    m_colorList.append(radialGradient);
}

void CustomButton::mousePressEvent(QMouseEvent *event)
{
    QPoint mousePressPoint = event->pos();
    QPoint translatePoint = mousePressPoint - QPoint(width() >> 1, height() >> 1);
    for (int i = 0; i < m_arcPathList.count(); i++)
    {
        if (m_arcPathList[i].contains(translatePoint) || m_textPathList[i].contains(translatePoint))
        {
            m_pressIndex = i;
            m_isMousePressed = true;
            //qDebug()<<"_________i = "<<i;
            update();
            emit signalButtonClicked(i);
            break;
        }
    }
}

void CustomButton::mouseReleaseEvent(QMouseEvent *event)
{
    Q_UNUSED(event);
    if (m_isMousePressed)
    {
        m_isMousePressed = false;
        mCenterRound = QPoint(0,0);
        mDegreePixmap = QPixmap(0,0);
        //qDebug()<<"_________m_pressIndex = "<<m_pressIndex;
        emit signalButtonReleased(m_pressIndex);
        update();
    }
}

void CustomButton::mouseMoveEvent(QMouseEvent *event)
{
    if(m_isMousePressed)
    {
        mCenterRound = event->pos() - QPoint(width() >> 1, height() >> 1);

        int x = mCenterRound.x();
        int y = -mCenterRound.y();
        int angle = analysisAngle(x,y);

        if(angle > 45 && angle <= 135 ){
            if(mAxesVertical == false)
                mCenterRound.setX(0);
            mCurWorkRegion = QUADRANT_UP;
        }else if(angle > 135 && angle <= 225){
            if(mAxesVertical == false)
            mCenterRound.setY(0);
            mCurWorkRegion = QUADRANT_LEFT;
        }else if(angle > 225 && angle <= 315){
            if(mAxesVertical == false)
            mCenterRound.setX(0);
            mCurWorkRegion = QUADRANT_DOWN;
        }else{
            if(mAxesVertical == false)
            mCenterRound.setY(0);
            mCurWorkRegion = QUADRANT_RIGHT;
        }

        int degree = qSqrt(qPow(x,2)+qPow(y,2));
        qDebug()<<degree;

        mDegreePixmap = getPixmap(degree);

        update();
    }
}
/*************************************************
Function:       analysisAngle
Description:    获取当前鼠标的度数
Input:          x,y
Return:         int,当前鼠标的度数
*************************************************/
int CustomButton::analysisAngle(int x,int y)
{
    double angle;

    if(x == 0 && y > 0)
    {
        return 90;
    }
    else if(x == 0 && y < 0)
    {
        return 270;
    }
    else if(x >= 0 && y == 0)
    {
        return 0;
    }
    else if(x < 0 && y == 0)
    {
        return 180;
    }
    else
    {
        angle = atan2(qAbs(y),qAbs(x))/(2*acos(-1))*360;
        if(x < 0 && y > 0)
        {
            angle = 180 - angle;
        }
        if(x < 0 && y < 0)
        {
            angle += 180;
        }
        if(x > 0 && y< 0)
        {
            angle = 360 - angle;
        }
        return angle;
    }
}




QPixmap CustomButton::getPixmap(const int ping)
{
    return getSignalPixmap(QColor(0x5,0x00c7,0xc7),getLineNum(ping));
}

QColor CustomButton::getColor(const int ping )
{
    if(ping<=10)
        return QColor(0xea,0x00,0x00);
    else if(ping<=20)
        return QColor(0xff,0x00,0x80);
    else if(ping<=30)
        return QColor(0xe8,0x00,0xe8);
    else if(ping<=40)
        return QColor(0xea,0xc1,0x00);
    else if(ping<=50)
        return QColor(0xe1,0xe1,0x00);
    else if(ping<=60)
        return QColor(0x9a,0xff,0x02);
    else if(ping<=70)

        return QColor(0x00,0xff,0xff);
    else if(ping<=80)

        return QColor(0x28,0x94,0xff);
    else
        return QColor(0x6a,0x6a,0xff);
}

int CustomButton::getLineNum(const int ping)
{
    if(ping<=20)
        return 1;
    else if(ping<=40)
        return 2;
    else if(ping<=60)
        return 3;
    else if(ping<=80)
        return 4;
    else
        return 5;
}
QPixmap CustomButton::getSignalPixmap(const QColor &color,const int linenum)//获取信号位图
{
    QPixmap pixmap(60,30);
    pixmap.fill(QColor(255,255,255,0));
    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::HighQualityAntialiasing, true);
    painter.setPen(QPen(color,2,Qt::SolidLine,Qt::RoundCap,Qt::RoundJoin));

    //30 - i* 5 ，是为了顶部不要被削掉
    for(int i=1,xpos=0;i<=linenum;++i,++xpos)
    {
        painter.drawArc(30 - i * 6, 30 - i * 5, i * 12, i * 10, 53 * 16, 37 * 2 * 16);
    }
    return pixmap;
}
