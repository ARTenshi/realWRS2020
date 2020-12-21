import traceback
import os
import pickle

class SaveableObject(object):
    def __init__(self, name, default_value=None, save_dir='/tmp'):
        self._file_name = os.path.join(save_dir, name)
        self._value = None
        if os.path.exists(self._file_name):
            self.load()
        else:
            self._value = default_value

    def load(self):
        try:
            with open(self._file_name, 'rb') as f:
                self._value = pickle.load(f)
        except:
            traceback.print_exc()

    def save(self):
        try:
            with open(self._file_name, 'wb') as f:
                pickle.dump(self._value, f)
        except:
            traceback.print_exc()

    @property
    def value(self):
        return self._value
    
    @value.setter
    def value(self, value):
        self._value = value
        self.save()
