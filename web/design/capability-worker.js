'use strict';
self.onmessage = (event) => {
  if (event.data === '4p-probe') self.postMessage('4p-ready');
};
