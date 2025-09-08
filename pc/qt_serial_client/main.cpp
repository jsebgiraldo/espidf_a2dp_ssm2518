#include <QCoreApplication>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QByteArray>
#include <QTextStream>
#include <iostream>

static const quint8 SYNC_A = 0xAA;
static const quint8 SYNC_B = 0x55;

// Commands
enum : quint8 {
    CMD_WRITE_REG  = 0x01,
    CMD_READ_REG   = 0x02,
    CMD_WRITE_BURST= 0x03,
    CMD_READ_BURST = 0x04,
    CMD_PING       = 0x05,
    CMD_TLV_DETECT = 0x06,
    RSP_WRITE_ACK  = 0x81,
    RSP_READ_VAL   = 0x82,
    RSP_WRITE_BURST_ACK = 0x83,
    RSP_READ_BURST_VAL  = 0x84,
    RSP_PONG       = 0x85,
    RSP_TLV_DETECT = 0x86,
    RSP_ERROR      = 0xE0,
};

static quint8 cks(quint8 cmd, quint8 len, const QByteArray &payload) {
    quint32 s = cmd + len;
    for (int i = 0; i < len && i < payload.size(); ++i) s += quint8(payload[i]);
    return quint8(-qint32(s));
}

static bool sendFrame(QSerialPort &sp, quint8 cmd, const QByteArray &payload) {
    QByteArray frame;
    frame.reserve(4 + payload.size() + 1);
    frame.push_back(char(SYNC_A));
    frame.push_back(char(SYNC_B));
    frame.push_back(char(cmd));
    frame.push_back(char(payload.size() & 0xFF));
    frame.append(payload);
    frame.push_back(char(cks(cmd, payload.size() & 0xFF, payload)));
    qint64 w = sp.write(frame);
    if (w != frame.size()) return false;
    return sp.waitForBytesWritten(100);
}

static bool readFrame(QSerialPort &sp, quint8 &cmd, QByteArray &payload, int timeoutMs=200) {
    QByteArray buf;
    QElapsedTimer t; t.start();
    while (t.elapsed() < timeoutMs) {
        if (sp.waitForReadyRead(20)) buf += sp.readAll();
        // scan
        for (int i = 0; i + 4 <= buf.size(); ++i) {
            if ((quint8)buf[i] != SYNC_A) continue;
            if ((quint8)buf[i+1] != SYNC_B) continue;
            quint8 c = (quint8)buf[i+2];
            quint8 l = (quint8)buf[i+3];
            int total = 4 + l + 1;
            if (i + total > buf.size()) break; // need more
            QByteArray pl = buf.mid(i+4, l);
            quint8 sum = (quint8)buf[i+4+l];
            if (sum == cks(c, l, pl)) {
                cmd = c; payload = pl;
                // discard up to end of this frame
                buf.remove(0, i + total);
                return true;
            } else {
                buf.remove(0, i+1); // resync
                break;
            }
        }
    }
    return false;
}

static bool openPort(QSerialPort &sp, const QString &name) {
    sp.setPortName(name);
    sp.setBaudRate(QSerialPort::Baud115200);
    sp.setDataBits(QSerialPort::Data8);
    sp.setParity(QSerialPort::NoParity);
    sp.setStopBits(QSerialPort::OneStop);
    sp.setFlowControl(QSerialPort::NoFlowControl);
    return sp.open(QIODevice::ReadWrite);
}

static int cmdPing(QSerialPort &sp) {
    QByteArray pl; // empty or version
    if (!sendFrame(sp, CMD_PING, pl)) return 1;
    quint8 rc=0; QByteArray rpl;
    if (!readFrame(sp, rc, rpl)) return 2;
    if (rc != RSP_PONG || rpl.size() < 4) return 3;
    QTextStream(stdout) << "PONG proto=" << (int)(quint8)rpl[0] << "." << (int)(quint8)rpl[1]
                        << " feat=0x" << QString::number((quint8)rpl[2],16).rightJustified(2,'0')
                        << ("" ) << Qt::endl;
    return 0;
}

static int cmdDetect(QSerialPort &sp) {
    QByteArray pl;
    if (!sendFrame(sp, CMD_TLV_DETECT, pl)) return 1;
    quint8 rc=0; QByteArray rpl;
    if (!readFrame(sp, rc, rpl)) return 2;
    if (rc != RSP_TLV_DETECT || rpl.size() < 2) return 3;
    bool present = (quint8)rpl[0] != 0;
    quint8 addr  = (quint8)rpl[1];
    QTextStream(stdout) << "TLV present=" << (present?"yes":"no") << " addr=0x"
                        << QString::number(addr,16).rightJustified(2,'0') << Qt::endl;
    return 0;
}

static int cmdRead(QSerialPort &sp, quint8 page, quint8 reg) {
    QByteArray pl; pl.push_back(char(page)); pl.push_back(char(reg));
    if (!sendFrame(sp, CMD_READ_REG, pl)) return 1;
    quint8 rc=0; QByteArray rpl;
    if (!readFrame(sp, rc, rpl)) return 2;
    if (rc == RSP_ERROR) {
        QTextStream(stdout) << "ERROR cmd=READ" << Qt::endl; return 3;
    }
    if (rc != RSP_READ_VAL || rpl.size() < 3) return 4;
    quint8 val = (quint8)rpl[2];
    QTextStream(stdout) << "P" << (int)rpl[0] << " R0x" << QString::number((quint8)rpl[1],16).rightJustified(2,'0')
                        << " = 0x" << QString::number(val,16).rightJustified(2,'0') << Qt::endl;
    return 0;
}

static int cmdWrite(QSerialPort &sp, quint8 page, quint8 reg, quint8 val) {
    QByteArray pl; pl.push_back(char(page)); pl.push_back(char(reg)); pl.push_back(char(val));
    if (!sendFrame(sp, CMD_WRITE_REG, pl)) return 1;
    quint8 rc=0; QByteArray rpl;
    if (!readFrame(sp, rc, rpl)) return 2;
    if (rc != RSP_WRITE_ACK || rpl.size() < 3) return 3;
    bool ok = (quint8)rpl[2] != 0;
    QTextStream(stdout) << (ok?"OK":"ERR") << Qt::endl;
    return ok ? 0 : 4;
}

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);
    QTextStream qout(stdout), qerr(stderr);
    QStringList a = app.arguments();
    if (a.size() < 3) {
        qerr << "Usage: qt_serial_client <COMx> <ping|detect|read|write> [args...]" << Qt::endl;
        return 1;
    }
    QString port = a[1];
    QString cmd  = a[2].toLower();

    QSerialPort sp;
    if (!openPort(sp, port)) { qerr << "Cannot open port " << port << Qt::endl; return 2; }

    int rc = 0;
    if (cmd == "ping") rc = cmdPing(sp);
    else if (cmd == "detect") rc = cmdDetect(sp);
    else if (cmd == "read" && a.size() >= 5) {
        bool ok1=false, ok2=false;
        int page = a[3].toInt(&ok1, 0);
        int reg  = a[4].toInt(&ok2, 0);
        if (!ok1 || !ok2) rc = 3; else rc = cmdRead(sp, quint8(page), quint8(reg));
    } else if (cmd == "write" && a.size() >= 6) {
        bool ok1=false, ok2=false, ok3=false;
        int page = a[3].toInt(&ok1, 0);
        int reg  = a[4].toInt(&ok2, 0);
        int val  = a[5].toInt(&ok3, 0);
        if (!ok1 || !ok2 || !ok3) rc = 3; else rc = cmdWrite(sp, quint8(page), quint8(reg), quint8(val));
    } else {
        qerr << "Bad args" << Qt::endl; rc = 4;
    }
    sp.close();
    return rc;
}
