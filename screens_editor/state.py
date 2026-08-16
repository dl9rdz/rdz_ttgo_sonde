class State:
    def __init__(self):
        self.variables = {"lat": 63.92812,"lon": -18.81629,"alt": 27178.3,"vs": 5.4,"hs": 54.3,"climb": 5.4,"speed": 54.3,"dir": 76.8,"temp": -79.8,"humidity": 8.3,"type":"RS41-SG","id": "W3551283","ser": "W3551283","frame": 7151,"vframe": 7151,"time": 1734830312,"sats": 11,"freq": 403.00,"rssi": 255,"afc": -427,"launchKT": 65535,"burstKT": 30600,"countKT": 65535,"crefKT": 7140,"launchsite": "KEF","res": 1,"batt": 2.2,"active":1,"validId":1,"validPos":127, "ip": "192.168.199.199", "version_name": "rdzTTGOsonde", "version_id": "dev20200101"}

    def get(self, key, defvalue=None):
        return self.variables.get(key, defvalue)

    def set(self, key, value):
        self.variables[key] = value

