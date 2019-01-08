#include "QLogger.h"

QLogger::QLogger(QTextEdit * _textEdit) :
   textEdit(_textEdit),
   Logger()
{
}

void QLogger::log(std::string str)
{
   textEdit->append(QString::fromStdString(str));
}

void QLogger::log(std::stringstream & ss)
{
// log(ss.str());
   textEdit->append(QString::fromStdString(ss.str()));
}
