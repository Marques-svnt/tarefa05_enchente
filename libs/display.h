#ifndef DISPLAY_H
#define DISPLAY_H

// Declaração da função de display
void initI2C();
void display(const char *texto, int x, int y);
void limpar();

#endif