import numpy as np
from sensor_msgs.msg import Image


def imgmsg_to_bgr8(msg):
    enc = msg.encoding.lower()
    channels = 1 if enc in ('mono8', '8uc1') else 3
    dtype = np.uint8
    arr = np.frombuffer(msg.data, dtype=dtype)
    if channels == 1:
        arr = arr.reshape(msg.height, msg.step)[:, :msg.width]
        return np.dstack([arr, arr, arr]).copy()
    row_width = msg.width * channels
    arr = arr.reshape(msg.height, msg.step)[:, :row_width].reshape(msg.height, msg.width, channels)
    if enc in ('rgb8', 'rgba8'):
        return arr[:, :, :3][:, :, ::-1].copy()
    if enc in ('bgr8', 'bgra8'):
        return arr[:, :, :3].copy()
    if enc in ('8uc3',):
        return arr.copy()
    raise ValueError('unsupported image encoding: %s' % msg.encoding)


def bgr8_to_imgmsg(image, header=None):
    msg = Image()
    if header is not None:
        msg.header = header
    msg.height = int(image.shape[0])
    msg.width = int(image.shape[1])
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = int(image.shape[1] * 3)
    msg.data = np.ascontiguousarray(image).tobytes()
    return msg
