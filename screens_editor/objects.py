
from drawing_area import mylcdfonts, myfonts


class Object:
    def __init__(self, font, x, y, xscale, yscale, width=0, extra=None, fg=(1, 1, 1), bg=None):
        self.font = font
        self.x = x
        self.y = y
        self.xscale = xscale
        self.yscale = yscale
        self.extra = extra
        self.fg = fg
        self.bg = bg
        self.width = width

    def render(self, canvas, state, area_height, scale, offset_x=0, offset_y=0):
        raise NotImplementedError("Subclasses must implement this method")


def create_object_class(name, format_logic):
    """
    Dynamically creates an object class with specific formatting logic.
    :param name: Name of the class.
    :param format_logic: A callable for formatting the value.
    :return: A dynamically created class.
    """

    class DynamicObject(Object):
        def render(self, canvas, state, area_height, scale=1, islcd=False, offset_x=0, offset_y=0):
            """
            Render the object on the canvas.
            """
            text = format_logic(state, self.extra)
            if islcd:
                fonts = mylcdfonts
                x = self.x * 8 * scale + offset_x
                y = self.y * 8 * scale + offset_y
                w = self.width * 8
            else:
                fonts = myfonts
                x = self.x * self.xscale * scale + offset_x
                y = self.y * self.yscale * scale + offset_y
                w = self.width * self.xscale
            if self.font >= len(fonts):
                print("Cannot render text: invalid font")
                return
            fonts[self.font].render_text(canvas, text, x, y, scale, width=w, fg=self.fg, bg=self.bg,
                                      area_height=area_height)

    DynamicObject.__name__ = name
    return DynamicObject


def format_text(state, extra):
    print("Printing text ", extra)
    return f"{extra}"


def format_latitude(state, extra):
    value = state.get("lat", 0)
    return f"{value:2.5f}"


def format_longitude(state, extra):
    value = state.get("lon", 0)
    return f"{value:2.5f}"


def format_altitude(state, extra):
    value = state.get("alt", 0)
    if value >= 1000:
        return f"   {value:5.0f}m"[-6:]  # Right-align
    else:
        return f"   {value:3.1f}m"[-6:]


def format_hs(state, extra):
    value = state.get("hs")
    is_ms = extra and extra[0] == "m"
    if not is_ms:
        value = value * 3.6  # Convert m/s to km/h
    text = f"{value:3.0f}" if value > 99 else f"{value:2.1f}"
    if extra and len(extra) > 1:
        text += extra[1:]
    return text[-(4 + len(extra[1:]) if extra and len(extra) > 1 else 4):]


def format_vs(state, extra):
    value = state.get("vs")
    text = f"  {value:+2.1f}"
    if extra:
        text += extra
    return text[-(5 + len(extra) if extra else 5):]


def format_id(state, extra):
    valid_id = state.get("validId", 0)
    sonde_id = state.get("id", "nnnnnnnn")
    serial = state.get("ser", "nnnnnnnn")
    sonde_type = state.get("type", "")

    if not valid_id:
        return "nnnnnnnn "
    elif extra and extra[0] == 'n':
        return serial  # Real serial number
    elif extra and extra[0] == 's':
        if sonde_type.startswith("DFM"):
            return sonde_id[1:]  # Skip initial "D"
        elif sonde_type.startswith("M"):
            return f"M{sonde_id[2:9]}"  # Replace "ME" with "M"
        else:
            return sonde_id
    else:
        return sonde_id  # Default case


def format_rssi(state, extra):
    rssi = state.get("rssi", 0)
    disptype = state.get("disptype", 0)

    decimal = '5' if rssi & 1 else '0'
    return f"-{rssi // 2}.{decimal}  "


def format_type(state, extra):
    sonde_type = state.get("type", "")
    return sonde_type


def format_freq(state, extra):
    freq = state.get("freq", 0.0)
    print("freq is ", freq)
    return f"{freq:3.3f}{extra or ''}"

def format_site(state, extra):
    if extra and extra[0]=='#':
        return "12" + extra[1:]
    else:
        return state.get("launchsite", "")

def format_ip(state, extra):
    return state.get("ip")

def format_afc(state, extra):
    afc = state.get("afc", 0.0)
    return f"     {afc * 0.001:+3.2f}k"[-8:]  # Format and right-align

def format_version(state, extra):
    if extra and extra[0] == 'v':
        return state.get("version_id")
    if extra and extra[0] == 's':
        return state.get("version_name")

object_classes = {
    "TextObject": create_object_class("TextObject", format_text),
    "LatitudeObject": create_object_class("LatitudeObject", format_latitude),
    "LongitudeObject": create_object_class("LongitudeObject", format_longitude),
    "AltitudeObject": create_object_class("AltitudeObject", format_altitude),
    "HSObject": create_object_class("HSObject", format_hs),
    "VSObject": create_object_class("VSObject", format_vs),
    "IDObject": create_object_class("IDObject", format_id),
    "RSSIObject": create_object_class("RSSIObject", format_rssi),
    "TypeObject": create_object_class("TypeObject", format_type),
    "FreqObject": create_object_class("FreqObject", format_freq),
    "SiteObject": create_object_class("SiteObject", format_site),
    "AFCObject": create_object_class("AFCObject", format_afc),
    "IPObject": create_object_class("IPObject", format_ip),
    "VersionObject": create_object_class("VersionObject", format_version),
}
