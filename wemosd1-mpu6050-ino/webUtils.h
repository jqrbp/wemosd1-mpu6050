#ifndef _WEB_UTILS_H_
#define _WEB_UTILS_H_

//#define WEB_WIFI_CLIENT
void ws_send_txt(String _str);
void web_init(const char *_APID, const char *_APPASS);
void web_loop(void);

#endif