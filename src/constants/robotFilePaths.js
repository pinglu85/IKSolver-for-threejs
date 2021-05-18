const KUKA_URL = './urdf/kuka_lwr/urdf/kuka_lwr.URDF';
const STAUBLI_URL =
  './urdf/staubli_tx2_90/staubli_tx2_90_support/urdf/tx2_90.xacro';
const STAUBLI_PACKAGES_URLS = {
  staubli_resources:
    'http://localhost:1234/urdf/staubli_tx2_90/staubli_resources/',
  staubli_tx2_90_support:
    'http://localhost:1234/urdf/staubli_tx2_90/staubli_tx2_90_support/',
};

export { KUKA_URL, STAUBLI_URL, STAUBLI_PACKAGES_URLS };
