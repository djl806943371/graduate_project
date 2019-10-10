# qt自定义插件，将库文件复制到 /home/Qt/Tools/QtCreator/lib/Qt/plugins/designer 下
# cmake项目，cpp文件中包含ui文件，必须把下面这句放在cpp文件第一行，天坑，这特么算qt跟cmake的兼容性bug了啊：
    #include "ui_settingsdialog.h"