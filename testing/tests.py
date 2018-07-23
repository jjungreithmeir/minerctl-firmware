import pytest
import serial
import time
import random

SERIAL = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

def _read(cmd):
    _write(cmd)
    while not SERIAL.in_waiting:
        time.sleep(.1)
    return SERIAL.read(SERIAL.in_waiting).decode('utf-8').strip()

def _write(cmd):
    resp_len = SERIAL.write(bytes(cmd + '\n', encoding='utf-8'))
    return resp_len

def _is_int(s):
    try:
        int(s)
        return True
    except ValueError:
        return False

def test_fw():
    assert _read("?fw") == "0.2"

def test_pidp():
    val = str(random.randint(0, 255))
    _write("!pidp {}".format(val))
    assert _read("?pidp") == val

def test_pidi():
    val = str(random.randint(0, 255))
    _write("!pidi {}".format(val))
    assert _read("?pidi") == val

def test_pidd():
    val = str(random.randint(0, 255))
    _write("!pidd {}".format(val))
    assert _read("?pidd") == val

def test_pidb():
    val = str(random.randint(0, 255))
    _write("!pidb {}".format(val))
    assert _read("?pidb") == val

def test_target_temp():
    val = str(random.randint(0, 255))
    _write("!targettemp {}".format(val))
    assert _read("?targettemp") == val

def test_minrpm():
    val = str(random.randint(1, 30))
    _write("!minrpm {}".format(val))
    assert _read("?minrpm") == val

def test_maxrpm():
    val = str(random.randint(60, 99))
    _write("!maxrpm {}".format(val))
    assert _read("?maxrpm") == val

def test_external():
    val = str(random.randint(0, 255))
    _write("!external {}".format(val))
    assert _read("?external") == val

def test_ontime():
    val = str(random.randint(0, 2500))
    _write("!ontime {}".format(val))
    assert _read("?ontime") == val

def test_offtime():
    val = str(random.randint(0, 2500))
    _write("!offtime {}".format(val))
    assert _read("?offtime") == val

def test_restime():
    val = str(random.randint(0, 2500))
    _write("!restime {}".format(val))
    assert _read("?restime") == val

def test_mode():
    val = str(random.randint(0, 1))
    _write("!mode {}".format(val))
    assert _read("?mode") == val

def test_threshold():
    val = str(random.randint(0, 2500))
    _write("!threshold {}".format(val))
    assert _read("?threshold") == val

def test_sensor():
    val = str(random.randint(0, 3))
    _write("!sensor {}".format(val))
    assert _read("?sensor") == val

def test_frequency():
    val = str(random.randint(0, 36))
    _write("!frequency {}".format(val))
    assert _read("?frequency") == val

def test_filter():
    filter_status = _read("?filter")
    assert _is_int(filter_status) and (filter_status in ("0", "1"))

def test_rpm():
    assert _is_int(_read("?rpm"))

def test_pressure():
    assert _is_int(_read("?pressure"))

def test_miner():
    rand_id = str(random.randint(0, 119))
    actions = {"deregister": -1, "register": 1, "on": 1, "off": 0}
    action = random.choice(list(actions))
    _write("!miner {} {}".format(rand_id, action))
    assert _read("?miner {}".format(rand_id)) == str(actions[action])

def test_commit():
    assert _read("!commit") == "Changes committed."
