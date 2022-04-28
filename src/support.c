#include "stm32f0xx.h"
#include <stdint.h>
#include <stdlib.h>
#include "lcd.h"
#include "midi.h"
#include "midiplay.h"

//DJIKSTRA
typedef struct _Node    {
    short value;
    short coordinate_x;
    short coordinate_y;
    int prev_row;
    int prev_col;
    int visited;            //BOOL TO INT
    short s_d;  //shortest distance
    struct _Node *north;
    struct _Node *south;
    struct _Node *east;
    struct _Node *west;
    struct _Node *next; //for writing to fastest times
} Node;

typedef struct _Coord   {
    int row;
    int column;
} coord;

//DJIKSTRA

static void nano_wait(unsigned int n) {
    asm(    "        mov r0,%0\n"
            "repeat: sub r0,#83\n"
            "        bgt repeat\n" : : "r"(n) : "r0", "cc");
}

void drive_column(int c)
{
    GPIOC->BSRR = 0xf00000 | ~(1 << (c + 4));
}

int read_rows()
{
    return (~GPIOC->IDR) & 0xf;
}

// Copy a subset of a large source picture into a smaller destination.
// sx,sy are the offset into the source picture.
void pic_subset(Picture *dst, const Picture *src, int sx, int sy)
{
    int dw = dst->width;
    int dh = dst->height;
    for(int y=0; y<dh; y++) {
        if (y+sy < 0)
            continue;
        if (y+sy >= src->height)
            break;
        for(int x=0; x<dw; x++) {
            if (x+sx < 0)
                continue;
            if (x+sx >= src->width)
                break;
            dst->pix2[dw * y + x] = src->pix2[src->width * (y+sy) + x + sx];
        }
    }
}

// Overlay a picture onto a destination picture.
// xoffset,yoffset are the offset into the destination picture that the
// source picture is placed.
// Any pixel in the source that is the 'transparent' color will not be
// copied.  This defines a color in the source that can be used as a
// transparent color.
void pic_overlay(Picture *dst, int xoffset, int yoffset, const Picture *src, int transparent)
{
    for(int y=0; y<src->height; y++) {
        int dy = y+yoffset;
        if (dy < 0)
            continue;
        if (dy >= dst->height)
            break;
        for(int x=0; x<src->width; x++) {
            int dx = x+xoffset;
            if (dx < 0)
                continue;
            if (dx >= dst->width)
                break;
            unsigned short int p = src->pix2[y*src->width + x];
            if (p != transparent)
                dst->pix2[dy*dst->width + dx] = p;
        }
    }
}

extern const Picture background; // A 240x320 background image
extern const Picture ball; // A 19x19 purple ball with white boundaries

//CAT
extern const Picture cat;   //THE CAT
extern const Picture wall;


// This C macro will create an array of Picture elements.
// Really, you'll just use it as a pointer to a single Picture
// element with an internal pix2[] array large enough to hold
// an image of the specified size.
// BE CAREFUL HOW LARGE OF A PICTURE YOU TRY TO CREATE:
// A 100x100 picture uses 20000 bytes.  You have 32768 bytes of SRAM.
#define TempPicturePtr(name,width,height) Picture name[(width)*(height)/6+2] = { {width,height,2} }

void erase(int x, int y)
{
    TempPicturePtr(tmp,29,29); // Create a temporary 29x29 image.
    pic_subset(tmp, &background, x-tmp->width/2, y-tmp->height/2); // Copy the background
    //pic_overlay(tmp, 5,5, &ball, 0xffff); // Overlay the ball
    LCD_DrawPicture(x-tmp->width/2,y-tmp->height/2, tmp); // Draw
}

void update(int x, int y)
{
    LCD_DrawPicture(x-ball.width/2,y-ball.height/2, &ball); // Draw the ball

    //CAT
    LCD_DrawPicture(x-cat.width/2, y-cat.height/2, &cat);
}

void update2(int x, int y)
{
    TempPicturePtr(tmp,20,20); // Create a temporary 29x29 image.
    pic_subset(tmp, &background, x-tmp->width/2, y-tmp->height/2); // Copy the background
    pic_overlay(tmp, 0,0, &ball, 0xffff); // Overlay the ball

    LCD_DrawPicture(x-tmp->width/2,y-tmp->height/2, tmp); // Draw
}

void update3(int x, int y)
{
    //CAT
    /*
    TempPicturePtr(tmp,29,29); // Create a temporary 29x29 image.
    pic_subset(tmp, &background, x-tmp->width/2, y-tmp->height/2); // Copy the background
    pic_overlay(tmp, 5,5, &cat, 0xffff);
    LCD_DrawPicture(x-tmp->width/2,y-tmp->height/2, tmp); // Draw
    */
    TempPicturePtr(tmp,20,20); // Create a temporary 29x29 image.
    pic_subset(tmp, &background, x-tmp->width/2, y-tmp->height/2); // Copy the background
    pic_overlay(tmp,0 ,0, &cat, 0xffff);
    LCD_DrawPicture(x-tmp->width/2,y-tmp->height/2, tmp); // Draw
}

void update4(int matrix[16][12])  {

    for(int i = 1; i <= 14; i++) {
        for(int j = 1; j <= 10; j++)    {
            if(matrix[i][j] == 2)   {
                int coord_row = i * 20 + 10;
                int coord_col = j * 20 + 10;
                TempPicturePtr(tmp,20,20); // Create a temporary 29x29 image.
                //pic_subset(tmp, &background, coord_col-tmp->width/2, coord_row-tmp->height/2); // Copy the background
                //pic_overlay(tmp, 5,5, &wall, 0xffff);
                LCD_DrawPicture(coord_col-tmp->width/2,coord_row-tmp->height/2, tmp); // Draw
            }

        }
    }
}


//PRINTING DEBUGGER
#define FIFOSIZE 16
char serfifo[FIFOSIZE];
int seroffset = 0;

void enable_tty_interrupt(void) {
    //raise interrupt every time receive data register becomes not empty
    USART5->CR1 |= USART_CR1_RXNEIE;
    NVIC->ISER[0] = 1<<USART3_8_IRQn;

    //trigger a DMA operation every time the receive data register becomes not empty
    USART5->CR3 |= USART_CR3_DMAR;

    //enable RCC clock for DMA controller 2
    RCC->AHBENR |= RCC_AHBENR_DMA2EN;
    DMA2->RMPCR |= DMA_RMPCR2_CH2_USART5_RX;
    DMA2_Channel2->CCR &= ~DMA_CCR_EN;  // First make sure DMA is turned off

    //CMAR should be set to address of serfifo
    DMA2_Channel2->CMAR = &(serfifo);

    //CPAR should be set to the address of the USART5 RDR
    DMA2_Channel2->CPAR = &(USART5->RDR);

    //CNDTR should be set to FIFOSIZE
    DMA2_Channel2->CNDTR = FIFOSIZE;

    //The DIRection of copying should be from peripheral to memory.
    DMA2_Channel2->CCR &= ~DMA_CCR_DIR;

    //Neither the total-completion nor the half-transfer interrupt should be enabled.
    DMA2_Channel2->CCR &= ~DMA_CCR_HTIE;
    DMA2_Channel2->CCR &= ~DMA_CCR_TCIE;

    //Both the MSIZE and the PSIZE should be set for 8 bits.
    DMA2_Channel2->CCR &= ~DMA_CCR_MSIZE;
    DMA2_Channel2->CCR &= ~DMA_CCR_PSIZE;

    //MINC should be set to increment the CMAR.
    DMA2_Channel2->CCR |= DMA_CCR_MINC;

    //PINC should not be set so that CPAR always points at the USART5 RDR.
    DMA2_Channel2->CCR &= ~DMA_CCR_PINC;

    //Enable CIRCular transfers.
    DMA2_Channel2->CCR |= DMA_CCR_CIRC;

    //Do not enable MEM2MEM transfers.
    DMA2_Channel2->CCR &= ~DMA_CCR_MEM2MEM;

    //Set the Priority Level to highest.
    DMA2_Channel2->CCR |= DMA_CCR_PL;

    //Finally, make sure that the channel is enabled for operation.
    DMA2_Channel2->CCR |= DMA_CCR_EN;
}

void init_usart5(void){
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;  //enable port c
        RCC->AHBENR |= RCC_AHBENR_GPIODEN;  //enable port d

        //configure pin PC12 to be routed to USART5_TX
        GPIOC->MODER &= ~0x3000000;
        GPIOC->MODER |= 0x2000000;
        GPIOC->AFR[1] |= 2<<(4*4);  //second index -> pc12  (2 because AF2 pg 43 of BC data sheet)

        //configure pin PD2 to be routed to USART5_RX
        GPIOD->MODER &= ~0x30;
        GPIOD->MODER |= 0x20;
        GPIOD->AFR[0] |= 2<<(4*2);

        RCC->APB1ENR |= RCC_APB1ENR_USART5EN;   //enable RCC clock to USART5 peripheral
        USART5->CR1 &= ~USART_CR1_UE;   //first disable by turning off UE bit

        //set word size of 8 bits
        USART5->CR1 &= ~0x10000000;  //BIT 28: M1
        USART5->CR1 &= ~USART_CR1_M;

        //set it for one stop bit
        USART5->CR2 &= ~USART_CR2_STOP;

        //set it for no parity
        USART5->CR1 &= ~USART_CR1_PCE;

        //use 16x oversampling
        USART5->CR1 &= ~USART_CR1_OVER8;

        //use baud rate of 115200*
        USART5->BRR = 0x1a1;        //(pg 696 family reference)

        //enable transmitter and receiver by setting TE and RE bits
        USART5->CR1 |= USART_CR1_TE;
        USART5->CR1 |= USART_CR1_RE;

        //enable the USART
        USART5->CR1 |= USART_CR1_UE;

        //wait for TE and RE bits to be acknowledged
        while(!(USART5->ISR & USART_ISR_TEACK) || !(USART5->ISR & USART_ISR_REACK)){
        }
}

#include "fifo.h"
#include "tty.h"
#include <stdio.h>

short* read_write_values(int wall_mat[16][12])  {
    short * array = malloc(sizeof(short) * 140);        //create matrix of tiles

    //0th ROW IS WALL-LESS
    for(int i = 0; i < 10; i++) {
        array[i] = 1;
    }
    int k = 10;     //continue off frm first row
    for(int i = 1; i <= 12; i++)    {
        //iterate through rows
        for(int j = 0; j < 10; j++)   {
            //iterate through the columns
            if(j == 0 || j == 9)    {
                array[k] = 1;   //there are no walls in side columns
            }
            else    {
                if(wall_mat[i + 1][j + 1] == 2){
                    array[k] = 140;
                }
                else    {
                    array[k] = 1;
                }
            }
            k++;
        }
    }
    //13th ROW HAS NO WALLS
    for(int i = 130; i< 140; i++)   {
        array[i] = 1;
    }
    return array;
}

Node* relax_node(Node* node)    {
    short min = 32767;      //SHORT MAX IS 32767
    Node* key_return = NULL;    //The next node with the shortest distance to explore
    if(node -> north != NULL)   {
        Node* north_node = node -> north;
        if(node -> s_d + north_node -> value < north_node -> s_d)   {
            north_node -> s_d = node -> s_d + north_node -> value;
            north_node -> prev_row = node -> coordinate_x;
            north_node -> prev_col = node -> coordinate_y;
        }

        if((north_node -> s_d) < min && !(north_node -> visited))   {
            min = north_node -> s_d;
            key_return = north_node;
        }
    }
    if(node -> west != NULL)    {
        Node* west_node = node -> west;
        if(node -> s_d + west_node -> value < west_node -> s_d) {
            west_node -> s_d = node -> s_d + west_node -> value;
            west_node -> prev_row = node -> coordinate_x;
            west_node -> prev_col = node -> coordinate_y;
        }

        if(west_node -> s_d < min && !(west_node -> visited))   {
            min = west_node -> s_d;
            key_return = west_node;
        }
    }
    if(node -> east != NULL)    {
        Node* east_node = node -> east;

        if(node -> s_d + east_node -> value < east_node -> s_d) {
            east_node -> s_d = node -> s_d + east_node -> value;
            east_node -> prev_row = node -> coordinate_x;
            east_node -> prev_col = node -> coordinate_y;
        }

        if(east_node -> s_d < min && !(east_node -> visited))   {
            min = east_node -> s_d;
            key_return = east_node;
        }
    }
    if(node -> south != NULL)   {
        Node* south_node = node -> south;
        if(node -> s_d + south_node -> value < south_node -> s_d)   {
            south_node -> s_d = node -> s_d + south_node -> value;
            south_node -> prev_row = node -> coordinate_x;
            south_node -> prev_col = node -> coordinate_y;
        }

        if(south_node -> s_d < min && !(south_node -> visited)) {
            min = south_node -> s_d;
            key_return = south_node;
        }

    }
    node -> visited = 1;        //1 instead of true
    return key_return;
}

void relax_all(Node* parent_node)   {   //recurse through the graph
    int i;
    for(i = 0; i < 4; i++)  {
        Node* return_node = relax_node(parent_node);
        if(return_node != NULL) {
            relax_all(return_node);
        }
    }
}

void free_matrix(Node* matrix[14][10])  {
    for(int i = 0; i < 14; i++)   {
        for(int j = 0; j < 10; j++) {
            free(matrix[i][j]);
        }
    }
}

void disp_matrix(Node* matrix[14][10], int row, int col)  {
    int i;
    int j;
    for(i = 0; i < row; i++)    {
        for(j = 0; j < col; j++)    {
            printf("__");
            printf("|%d| ", matrix[i][j] -> value);
            printf("(%d ", matrix[i][j] -> coordinate_x);
            printf(",%d) ", matrix[i][j] -> coordinate_y);
            printf("v: %d ", matrix[i][j] -> visited);
            printf("sd: %d ", matrix[i][j] -> s_d);
            printf("prev:[%d,%d]", matrix[i][j] -> prev_row, matrix[i][j] -> prev_col);
        }
        printf("\n");
    }
}

void fastest_times(short* values, short cat_r, short cat_c, short mouse_r, short mouse_c, int* row_path, int* col_path)   {
    short i;
    short j;
    short k = 0;
    short rows = 14;
    short columns = 10;
    Node* matrix[rows][columns];
    for(i = 0; i < rows; i++)   {
            for(j = 0; j < columns; j++)    {
                Node* node = malloc(sizeof(*node));
                *node = (Node) {.value = values[k], .coordinate_x = i, .coordinate_y = j, .visited = 0, .s_d = 32767},
                matrix[i][j] = node;
                k++;
            }
    }
    for(i = 0; i < rows; i++)   {
            for(j = 0; j < columns; j++)    {
                if(i == 0)  {
                    //first row
                    matrix[i][j] -> north = NULL;
                }
                else    {
                    matrix[i][j] -> north = matrix[i - 1][j];
                }
                if(j == 0)  {
                    //first column
                    matrix[i][j] -> west = NULL;
                }
                else    {
                    matrix[i][j] -> west = matrix[i][j - 1];
                }
                if(i == rows - 1)   {
                    //last row
                    matrix[i][j] -> south = NULL;
                }
                else    {
                    matrix[i][j] -> south = matrix[i + 1][j];
                }
                if(j == columns - 1)    {
                    //last column
                    matrix[i][j] -> east = NULL;
                }
                else    {
                    matrix[i][j] -> east = matrix[i][j + 1];
                }
            }
        }
    Node* parent_node = matrix[mouse_r][mouse_c];
    parent_node -> s_d = parent_node -> value;
    relax_all(parent_node);
    grab_path(matrix, cat_r, cat_c, mouse_r, mouse_c, row_path, col_path);
    free_matrix(matrix);
}


void grab_path(Node* matrix[14][10], short c_r, short c_c, short m_r, short m_c, int* path_row, int* path_col)    {

    int ctr = 0;                    //column, row, row, column
    //printf("cat: %d, %d | mouse: %d, %d", c_r, c_c, m_r, m_c);

    while(c_r != m_r || c_c != m_c)   {
        //printf("cat path: (%d, %d)\n", c_x, c_y);

        Node* val = matrix[c_r][c_c];
        c_r = val -> prev_row;
        c_c = val -> prev_col;
        path_row[ctr] = c_r;      //switch
        path_col[ctr] = c_c;
        ctr++;

    }
    //PREVENT LOOSE ENDS of the array
    path_row[ctr] = -1;
    path_col[ctr] = -1;


}



int check_wall_new(int row, int col, int wall_row[96], int wall_col[96]) {
    int i = 0;
    int update = 1; //assume update
    while(update && wall_row[i] != -1)    {
        //for example for wall at (270, 170)
        //row > 250 && row < 290 && col > 150 && col < 190
        if(row > (wall_row[i] - 20) && row < (wall_row[i] + 20) && col > (wall_col[i] - 20) && col < (wall_col[i] + 20))    {
            update = 0;
        }
        i++;
    }
    return update;
}

int check_mouse(int row, int col)   {
    if(row > 290)  {
        //mouse is at entrance
        return 1;
    }
    else    {
        //mouse is not at entrance
        return 0;
    }
}

int check_mouse_exit(int row, int col)  {
    int exit = 0;
    if(row < 20)    {
        exit = 1;       //mouse has reached exit
    }
    return exit;
}

int cat_contact(int cat_row, int cat_col, int mouse_row, int mouse_col) {
    int touch = 0;
    if((abs(cat_row - mouse_row)  < 20) && abs(cat_col - mouse_col) < 20)    {
        touch = 1;
    }
    return touch;
}


void move_ball(void)
{
    int num_maps = 30;
    /*
     * EMPTY MAZE TEMPLATE
     int mat0[12][8] =   {{0,0,0,0,0,0,0,0},
                         {0,0,0,0,0,0,0,0},
                         {0,0,0,0,0,0,0,0},
                         {0,0,0,0,0,0,0,0},
                         {0,0,0,0,0,0,0,0},
                         {0,0,0,0,0,0,0,0},
                         {0,0,0,0,0,0,0,0},
                         {0,0,0,0,0,0,0,0},
                         {0,0,0,0,0,0,0,0},
                         {0,0,0,0,0,0,0,0},
                         {0,0,0,0,0,0,0,0},
                         {0,0,0,0,0,0,0,0}};
     */

                        //mat0- original maze
    int mat0[12*30][8] =   {{0,0,0,0,0,0,0,0},
                         {0,2,2,2,2,0,0,0},
                         {0,2,0,0,2,0,0,0},
                         {0,2,0,0,2,2,2,2},
                         {0,0,0,0,0,0,0,2},
                         {0,0,0,0,0,0,0,2},
                         {2,2,2,0,0,2,2,2},
                         {2,0,0,0,0,0,0,0},
                         {2,0,0,0,0,0,0,0},
                         {0,0,2,0,0,2,0,2},
                         {0,0,2,0,0,2,2,2},
                         {0,0,0,0,0,0,0,0},
                        //mat1- alien maze
                         {2,2,2,2,2,0,0,2},
                         {2,0,0,0,0,0,0,2},
                         {2,0,2,0,0,2,0,2},
                         {2,0,2,0,0,2,0,2},
                         {2,0,0,0,0,0,0,2},
                         {2,0,0,0,0,0,0,2},
                         {2,0,2,0,0,2,0,2},
                         {2,0,2,0,0,2,0,2},
                         {2,0,0,0,0,0,0,2},
                         {2,0,0,2,2,0,0,2},
                         {2,0,0,0,0,0,0,2},
                         {2,0,0,2,2,2,2,2},
                         //mat2- rick maze
                         {2,2,2,0,0,2,2,2},
                         {2,0,0,2,0,0,2,0},
                         {2,2,2,0,0,0,2,0},
                         {2,2,0,0,0,0,2,0},
                         {2,0,2,0,0,0,2,0},
                         {2,0,0,2,0,2,2,2},
                         {0,0,0,0,0,0,0,0},
                         {0,2,2,0,2,0,0,2},
                         {2,0,0,0,2,0,2,0},
                         {2,0,0,0,2,2,0,0},
                         {2,0,0,0,2,0,2,0},
                         {0,2,2,0,2,0,0,2},
                        //mat3- 362 maze
                        {2,2,0,0,0,2,2,2},
                        {0,0,2,0,0,2,0,0},
                        {2,2,0,0,0,2,2,2},
                        {0,0,2,0,0,2,0,2},
                        {2,2,0,0,0,2,2,2},
                        {0,0,0,0,0,0,0,0},
                        {0,0,0,0,0,0,0,0},
                        {2,2,2,0,0,2,0,2},
                        {0,0,2,0,0,2,0,2},
                        {2,2,2,0,0,2,0,2},
                        {2,0,0,0,0,0,0,0},
                        {2,2,2,0,0,2,0,2},
                        //mat4- tetris maze
                        {2,2,2,2,2,0,0,0},
                        {0,0,0,0,0,0,0,0},
                        {0,0,0,2,2,2,2,2},
                        {0,0,0,0,0,0,0,0},
                        {0,2,2,2,0,0,2,0},
                        {0,2,0,0,0,0,2,0},
                        {0,2,0,0,0,0,2,0},
                        {0,2,0,0,2,2,2,0},
                        {0,0,0,0,0,0,0,0},
                        {2,2,2,2,2,0,0,0},
                        {0,0,0,0,0,0,0,0},
                        {0,0,0,2,2,2,2,2},
                        //mat5- 69 maze
                        {2,0,2,2,2,2,2,2},
                        {2,0,2,0,2,0,0,2},
                        {2,0,2,0,2,0,0,2},
                        {2,0,2,0,2,0,0,2},
                        {2,0,2,2,2,0,0,2},
                        {2,0,0,0,0,0,0,0},
                        {2,0,0,0,0,0,0,0},
                        {2,0,0,0,0,2,2,2},
                        {2,0,0,0,0,2,0,2},
                        {2,0,0,0,0,2,0,2},
                        {2,0,0,0,0,2,0,2},
                        {2,0,2,2,2,2,2,2},
                        //mat6- zigzag maze
                        {2,2,2,2,2,0,0,2},
                        {2,0,0,0,0,0,0,2},
                        {2,0,2,2,2,2,2,2},
                        {2,0,0,0,0,0,0,2},
                        {2,2,2,2,2,2,0,2},
                        {2,0,0,0,0,0,0,2},
                        {2,0,2,2,2,2,2,2},
                        {2,0,0,0,0,0,0,2},
                        {2,2,2,2,2,2,0,2},
                        {2,0,0,0,0,0,0,2},
                        {2,0,2,0,0,0,0,2},
                        {2,0,0,0,2,2,2,2},
                        //mat7- smiley maze
                        {0,0,0,0,0,0,0,0},
                        {0,0,0,0,0,0,0,0},
                        {0,2,2,0,0,2,2,0},
                        {0,2,2,0,0,2,2,0},
                        {0,0,0,0,0,0,0,0},
                        {0,0,0,2,2,0,0,0},
                        {0,0,0,0,0,0,0,0},
                        {2,0,0,0,0,0,0,2},
                        {0,2,0,0,0,0,2,0},
                        {0,0,2,0,0,2,0,0},
                        {0,0,0,2,2,0,0,0},
                        {0,0,0,0,0,0,0,0},
                        //mat8- regular maze
                        {0,0,0,0,0,0,0,0},
                        {0,2,2,2,2,0,0,2},
                        {0,2,0,0,2,0,0,2},
                        {0,2,0,0,0,0,0,2},
                        {0,0,0,0,0,0,0,0},
                        {2,2,2,0,2,0,0,0},
                        {0,0,0,0,2,0,0,0},
                        {2,2,2,0,2,2,2,2},
                        {0,0,2,0,0,2,0,0},
                        {0,0,2,0,0,2,0,0},
                        {2,0,2,0,0,2,0,2},
                        {0,0,2,0,0,2,0,0},
                        //mat9 - horse maze
                        {2,2,2,0,2,2,0,0},
                        {0,0,0,0,0,0,0,0},
                        {0,0,2,0,0,2,0,0},
                        {0,0,2,0,0,2,0,0},
                        {0,0,0,0,0,0,0,0},
                        {0,0,2,0,0,0,0,0},
                        {0,0,2,2,0,2,2,2},
                        {0,0,0,0,0,0,0,0},
                        {0,0,2,2,2,2,0,0},
                        {0,0,0,0,0,0,0,2},
                        {0,0,0,0,0,0,0,2},
                        {0,0,0,0,0,0,2,2},
                        //mat10 - weird maze
                        {0,0,0,0,0,0,0,0},
                        {0,0,0,0,0,2,2,2},
                        {2,2,2,0,0,0,0,2},
                        {0,0,2,0,0,0,0,2},
                        {0,0,2,0,2,0,0,2},
                        {0,0,2,2,2,0,0,2},
                        {0,0,0,0,0,0,0,2},
                        {0,0,0,0,2,0,0,2},
                        {2,2,0,0,0,0,0,2},
                        {2,0,0,0,0,0,0,2},
                        {2,0,0,0,0,2,2,2},
                        {0,0,0,0,0,0,0,0},
                        //mat11 -cheng maze
                        {2,2,0,0,2,2,2,2},
                        {2,0,0,0,0,0,0,0},
                        {0,0,0,0,0,0,0,0},
                        {0,0,0,0,0,0,0,2},
                        {2,2,2,0,0,2,2,2},
                        {0,0,0,0,0,0,0,0},
                        {0,2,2,0,2,2,0,0},
                        {0,0,0,0,2,0,0,0},
                        {0,0,0,2,2,2,0,0},
                        {0,0,0,0,0,0,0,0},
                        {0,2,0,2,2,2,2,0},
                        {0,0,0,0,0,0,0,0},
                        //mat12 - ladder maze
                        {2,2,0,0,0,0,2,2},
                         {2,0,0,2,2,0,0,2},
                         {2,0,0,0,0,0,0,2},
                         {2,0,0,0,0,0,0,2},
                         {2,0,2,2,2,2,0,2},
                         {2,0,0,0,0,0,0,2},
                         {2,0,0,0,0,0,0,2},
                         {2,0,2,2,2,2,0,2},
                         {2,0,0,0,0,0,0,2},
                         {2,0,0,0,0,0,0,2},
                         {2,0,0,2,2,0,0,2},
                         {2,2,0,0,0,0,2,2},
                         //mat13 - funnel block maze
                         {2,2,2,0,0,2,2,2},
                          {0,0,2,0,0,2,0,0},
                          {0,0,2,0,0,2,0,0},
                          {2,2,2,0,0,2,2,2},
                          {2,0,0,0,0,0,0,2},
                          {2,0,2,2,2,2,0,2},
                          {2,0,2,2,2,2,0,2},
                          {2,0,0,0,0,0,0,2},
                          {2,2,2,0,0,2,2,2},
                          {0,0,2,0,0,2,0,0},
                          {0,0,2,0,0,2,0,0},
                          {2,2,2,0,0,2,2,2},
                          //mat14 - among us
                          {2,2,2,0,0,2,2,2},
                           {2,0,0,0,0,0,0,2},
                           {2,0,2,2,2,2,0,2},
                           {2,0,2,0,0,2,0,2},
                           {2,0,2,2,2,2,0,2},
                           {2,0,0,0,0,0,0,2},
                           {2,0,0,0,0,0,0,2},
                           {2,0,0,0,0,0,0,2},
                           {2,0,2,0,0,2,0,2},
                           {2,0,2,0,0,2,0,2},
                           {2,0,2,0,0,2,0,2},
                           {2,2,2,0,0,2,2,2},
                           //mat15- zigzag M
                           {2,2,2,2,2,2,2,0},
                            {2,0,0,0,0,0,0,0},
                            {2,0,2,2,2,2,2,2},
                            {2,0,0,0,0,0,0,2},
                            {2,2,2,2,2,2,0,2},
                            {0,0,0,0,0,0,0,2},
                            {0,0,0,0,0,0,0,2},
                            {2,2,2,2,2,2,0,2},
                            {2,0,0,0,0,0,0,2},
                            {2,0,2,2,2,2,2,2},
                            {2,0,0,0,0,0,0,0},
                            {2,2,2,2,02,2,2,0},
                            //mat16- blocks
                            {2,2,0,0,2,2,0,0},
                             {2,2,0,0,2,2,0,0},
                             {0,0,0,0,0,0,0,0},
                             {0,0,2,2,0,0,2,2},
                             {0,0,2,2,0,0,2,2},
                             {0,0,0,0,0,0,0,0},
                             {2,2,0,0,2,2,0,0},
                             {2,2,0,0,2,2,0,0},
                             {0,0,0,0,0,0,0,0},
                             {0,0,2,2,0,0,2,2},
                             {0,0,2,2,0,0,2,2},
                             {0,0,0,0,0,0,0,0},
                             //mat17 - square waves
                             {2,0,0,0,0,2,0,0},
                              {2,0,0,0,0,2,0,0},
                              {2,2,2,0,0,2,2,2},
                              {0,0,2,0,0,0,0,2},
                              {0,0,2,0,0,0,0,2},
                              {2,2,2,0,0,2,2,2},
                              {2,0,0,0,0,2,0,0},
                              {2,0,0,0,0,2,0,0},
                              {2,2,2,0,0,2,2,2},
                              {0,0,2,0,0,0,0,2},
                              {0,0,2,0,0,0,0,2},
                              {2,2,2,0,0,2,2,2},
                              //mat18- sin and cos
                              {2,0,0,0,0,0,0,2},
                               {0,2,0,0,0,0,0,2},
                               {0,0,2,0,0,0,2,0},
                               {0,0,2,0,0,2,0,0},
                               {0,2,0,0,0,2,0,0},
                               {2,0,0,0,0,0,2,0},
                               {2,0,0,0,0,0,0,2},
                               {0,2,0,0,0,0,0,2},
                               {0,0,2,0,0,0,2,0},
                               {0,0,2,0,0,2,0,0},
                               {0,2,0,0,0,2,0,0},
                               {2,0,0,0,0,0,2,0},
                               //mat19- PURDUE
                               {0,2,2,2,2,2,0,0},
                                {0,2,0,0,0,0,2,2},
                                {0,2,0,0,0,0,0,2},
                                {0,2,0,2,2,0,0,2},
                                {0,2,0,0,0,0,0,2},
                                {0,2,0,0,0,0,2,2},
                                {0,2,0,0,2,2,0,0},
                                {0,2,0,0,2,0,0,0},
                                {0,0,0,0,2,0,0,0},
                                {0,0,0,0,2,0,0,0},
                                {0,2,0,0,2,0,0,0},
                                {2,2,0,0,2,2,0,0},
                                //mat20- simple maze
                                {2,2,0,0,0,0,2,2},
                                 {2,0,0,0,0,0,0,2},
                                 {0,0,2,0,0,2,0,0},
                                 {0,0,2,0,0,2,0,0},
                                 {2,0,2,0,0,2,0,2},
                                 {2,0,0,0,0,0,0,2},
                                 {2,0,0,0,0,0,0,2},
                                 {2,0,2,0,0,2,0,2},
                                 {0,0,2,0,0,2,0,0},
                                 {0,0,2,0,0,2,0,0},
                                 {2,0,0,0,0,0,0,2},
                                 {2,2,0,0,0,0,2,2},
                                 //mat21- cat face
                                 {0,2,0,0,0,0,2,0},
                                  {2,0,2,0,0,2,0,2},
                                  {2,0,2,0,0,2,0,2},
                                  {0,0,0,0,0,0,0,0},
                                  {2,2,0,0,0,0,2,2},
                                  {0,0,0,0,0,0,0,0},
                                  {2,0,0,0,0,0,0,2},
                                  {0,0,2,2,2,2,0,0},
                                  {0,0,0,2,2,0,0,0},
                                  {0,0,0,2,2,0,0,0},
                                  {2,0,0,2,2,0,0,2},
                                  {2,2,2,2,2,2,2,2},
                                  //mat 22- grounded maze
                                  {0,0,0,2,2,0,0,0},
                                   {0,0,0,2,2,0,0,0},
                                   {0,0,0,2,2,0,0,0},
                                   {0,0,0,2,2,0,0,0},
                                   {0,0,0,2,2,0,0,0},
                                   {0,0,0,2,2,0,0,0},
                                   {0,0,0,2,2,0,0,0},
                                   {0,2,2,2,2,2,2,0},
                                   {0,0,0,0,0,0,0,0},
                                   {0,0,2,2,2,2,0,0},
                                   {0,0,0,0,0,0,0,0},
                                   {0,0,0,2,2,0,0,0},
                                   //mat 23- voltage supply
                                   {0,0,0,0,2,0,0,0},
                                    {0,2,0,0,2,0,0,0},
                                    {2,2,2,0,2,0,0,0},
                                    {0,2,0,0,2,0,0,0},
                                    {0,0,0,0,2,0,0,0},
                                    {0,0,0,0,2,0,0,0},
                                    {0,0,2,2,2,2,2,0},
                                    {0,0,0,0,0,0,0,0},
                                    {0,0,0,2,2,2,0,0},
                                    {0,0,0,0,2,0,0,0},
                                    {2,2,2,0,2,0,0,0},
                                    {0,0,0,0,2,0,0,0},
                                    //mat 24- ohms law
                                    {2,0,0,0,2,0,0,0},
                                     {2,0,0,0,2,0,0,0},
                                     {0,2,0,2,0,0,2,2},
                                     {0,2,0,2,0,0,0,0},
                                     {0,2,0,2,0,0,2,2},
                                     {0,0,0,0,0,0,0,0},
                                     {0,0,2,0,2,2,2,2},
                                     {0,0,0,0,2,0,0,2},
                                     {2,2,2,0,2,2,2,2},
                                     {0,2,0,0,2,2,0,0},
                                     {0,2,0,0,2,0,2,0},
                                     {2,2,2,0,2,0,0,2},
                                     //mat 25- stairs
                                     {0,0,0,2,2,0,0,0},
                                      {0,0,2,0,0,0,0,0},
                                      {0,2,0,0,0,0,0,0},
                                      {2,0,0,0,0,0,0,0},
                                      {2,2,0,0,0,0,0,0},
                                      {0,0,2,2,0,0,0,0},
                                      {0,0,0,0,2,2,0,0},
                                      {0,0,0,0,0,0,2,2},
                                      {0,0,0,0,2,2,0,0},
                                      {0,0,2,2,0,0,0,0},
                                      {2,2,0,0,0,0,0,0},
                                      {2,0,0,0,0,0,0,0},
                                      //mat 26- ECE
                                      {2,2,2,2,2,2,2,2},
                                       {2,0,0,2,2,0,0,2},
                                       {2,0,0,2,2,0,0,2},
                                       {0,0,0,0,0,0,0,0},
                                       {0,2,2,2,2,2,2,0},
                                       {0,2,0,0,0,0,2,0},
                                       {0,2,0,0,0,0,2,0},
                                       {0,2,0,0,0,0,2,0},
                                       {0,0,0,0,0,0,0,0},
                                       {2,2,2,2,2,2,2,2},
                                       {2,0,0,2,2,0,0,2},
                                       {2,0,0,2,2,0,0,2},
                                       //mat 27- derivative
                                       {0,0,2,0,2,0,0,2},
                                        {0,0,2,0,2,0,0,2},
                                        {0,0,2,0,2,2,2,2},
                                        {2,2,2,0,0,0,0,2},
                                        {2,0,2,0,0,0,0,2},
                                        {2,2,2,0,0,2,2,2},
                                        {0,0,0,0,0,0,0,0},
                                        {0,0,2,0,0,0,0,0},
                                        {0,0,2,0,2,0,0,2},
                                        {2,2,2,0,0,2,2,0},
                                        {2,0,2,0,0,2,2,0},
                                        {2,2,2,0,2,0,0,2},
                                        //mat 28- dead end
                                        {2,2,2,0,0,2,2,2},
                                         {2,0,0,0,0,2,0,2},
                                         {2,0,0,0,0,2,0,2},
                                         {2,0,2,0,0,2,0,2},
                                         {2,0,2,0,0,2,0,2},
                                         {2,0,2,0,0,2,0,2},
                                         {2,0,2,0,0,2,0,2},
                                         {2,0,2,0,0,2,0,2},
                                         {2,0,2,0,0,2,0,2},
                                         {2,0,2,0,0,0,0,2},
                                         {2,0,2,0,0,0,0,2},
                                         {2,2,2,0,0,2,2,2},
                                         //mat 29- inverse0
                                         {2,2,2,2,2,0,0,2},
                                          {2,0,0,0,0,0,0,2},
                                          {2,0,0,2,2,2,0,2},
                                          {2,0,0,0,0,2,0,2},
                                          {2,0,2,0,0,2,0,2},
                                          {2,0,2,0,0,2,0,2},
                                          {2,0,2,0,0,2,0,2},
                                          {2,0,2,0,0,2,0,2},
                                          {2,0,2,0,0,0,0,2},
                                          {2,0,2,2,2,0,0,2},
                                          {2,0,0,0,0,0,0,2},
                                          {2,0,0,2,2,2,2,2}

    };

    int game_over = 0;
    int difficulty;     //rate at which dijkstra updates cat path (CAT'S INTELLIGENCE)
    int num = -1;
    int mat[16][12];
    int last_maze = -1;     //keep track of last maze to prevent consecutive same random generations

    //fill in preliminary walls
    for(int i = 0; i < 16; i++) {
        for(int j = 0; j < 12; j++) {
            //deal with first two rows
            if(i == 0)  {
                if(j == 2 || j == 3)    {
                    //exit
                    mat[i][j] = 0;
                }
                else    {
                    mat[i][j] = 1;
                }
            }
            else if(i == 1) {
                if(j == 0 || j == 11)   {
                    mat[i][j] = 1;
                }
                else    {
                    mat[i][j] = 0;
                }
            }
            else if(i == 14)    {
                //deal with last two rows
                if(j == 0 || j == 11)   {
                    mat[i][j] = 1;
                }
                else    {
                    mat[i][j] = 0;
                }
            }
            else if(i == 15)    {
                if(j == 8 || j == 9)    {
                    mat[i][j] = 0;
                }
                else {
                    mat[i][j] = 1;
                }
            }
            //deal with first two columns
            if(j == 0)  {
                mat[i][j] = 1;
            }
            else if(j == 1) {
                if(i == 15 || i == 0) {
                    mat[i][j] = 1;
                }
                else    {
                    mat[i][j] = 0;
                }
            }
            else if(j == 10)    {
                //deal with last two columns
                if(i == 0 || i == 15)   {
                    mat[i][j] = 1;
                }
                else    {
                    mat[i][j] = 0;
                }
            }
            else if(j == 11)    {
                mat[i][j] = 1;
            }
        }
    }

    MIDI_Player *mp = midi_init(midifile);

    while(!game_over)   {
        num++;
        if(num <= 1)    {
            //level 1- easy
            difficulty = 750;
        }
        else if(num <= 3) {
            //levels 2 to 3
            difficulty = 500;
        }
        else if(num <= 5)   {
            //levels 4 to 5
            difficulty = 300;
        }
        else if(num <= 6)   {
            //levels 6
            difficulty = 150;
        }
        else    {
            //levels 6+ max difficulty
            difficulty = 75;
        }

        //prevent consecutive random maps
        int first_random = rand()%5;        //generate 0 to 4
        int random;
        switch(first_random)  {
            case(0):
                random = rand()%5;      //generate 0 to 4
                break;
            case(1):
                random = rand()%5 + 5;  //generate 5 to 9
                break;
            case(2):
                random = rand()%5 + 10; //generates 10 to 14
                break;
            case(3):
                random = rand()%5 + 15; //generates 15 to 19
                break;
            case(4):
                random = rand()%5 + 20; //generates 20 to 24
                break;
            default:
                random = rand()%5 + 25; //generates 25 to 29
                break;
        }

        if(last_maze == -1) {
            last_maze = random;
        }
        else    {
            while(last_maze == random)  {
                random = rand()%num_maps;
            }
        }
        last_maze = random;

        for(int i = (2 + random * 12); i <= (2 + random * 12 + 11); i++)    {
            for(int j = 2; j <= 9; j++) {
                mat[i - random * 12][j] = mat0[i - 2][j - 2];
            }
        }

        //Score the map
        char scoreNum[5];
        itoa(num, scoreNum, 10);

        //the following arrays hold all coordinates to black walls
        int black_wall_row[96];     //96 is 12 * 8 where the walls possibly lie
        int black_wall_col[96];

        for(int i = 0; i < 96; i++) {
            //fill with -1s
            black_wall_row[i] = -1;
            black_wall_col[i] = -1;
        }

        int k = 0;

        for(int i = 2; i < 14; i++) {
            //check rows 2 to 13
            for(int j = 2; j < 10; j++) {
                //check columns 2 to 10
                if(mat[i][j] == 2)  {
                    //there is a black wall
                    black_wall_row[k] = i * 20 + 10;
                    black_wall_col[k] = j * 20 + 10;
                    k++;
                }
            }
        }


        LCD_DrawPicture(0,0,&background);

        int mouse_col = 180;            //mouse column
        int mouse_row = 310;            //mouse row
        int cat_col = 210;
        int cat_row = 30;
        int cat_idle_row = 30;
        int cat_idle_col = 50;
        int cat_retreat = 0;        //checks if cat is returning to entrance
        int mouse_entrance = 1;         //checks if mouse is at entrance (1 for yes, 0 for no)

        update2(mouse_col,mouse_row);
        update3(cat_col, cat_row);

        //make preliminary cat path
        int path_row [140];
        int path_col [140];
        for(int i = 0; i < 140; i++)    {
            path_row[i] = -1;
            path_col[i] = -1;
        }

        int counter = 0;
        int path_progress = 0;
        update4(mat);

        for(;;){
            LCD_DrawString(150,1, BLACK, WHITE, "Score: ", 12, 0);
            LCD_DrawString(190,1, BLACK, WHITE, scoreNum, 12, 0);

            if (mp->nexttick == MAXTICKS)
                mp = midi_init(midifile);

            if(cat_contact(cat_row, cat_col, mouse_row, mouse_col)) {
                game_over = 1;
                break;
            }
            if(check_mouse_exit(mouse_row, mouse_col))  {
                game_over = 0;
                break;
            }

            mouse_entrance = check_mouse(mouse_row, mouse_col);
            if(mouse_entrance && !cat_retreat)  {
                //if mouse is at entrance send cat into default state
                path_progress = 0;
                short*matrix = read_write_values(mat);
                fastest_times(matrix, (int)((cat_row - 30) / 20) , (int)(cat_col - 30) / 20, (int)((cat_idle_row - 30) / 20), (int)(cat_idle_col - 30) / 20, path_row, path_col);
                free(matrix);
                cat_retreat = 1;
            }

            //DIJKSTRA
            if(counter % difficulty == 0 && !mouse_entrance)   {
                path_progress = 0;
                cat_retreat = 0;
                short* matrix = read_write_values(mat);    //hard code of the walls

                fastest_times(matrix, (int)((cat_row - 30) / 20) , (int)(cat_col - 30) / 20, (int)(mouse_row - 30) / 20, (int) (mouse_col - 30) / 20, path_row, path_col);
                free(matrix);

            }

            if(path_row[path_progress] != -1) {


                int go_to_row = 30 + 20 * path_row[path_progress];
                int row_vector = go_to_row - cat_row;

                int go_to_column = 30 + 20 * path_col[path_progress];
                int column_vector = go_to_column - cat_col;

                if(column_vector)   {
                    if(column_vector > 0)    {
                        //move positive rows
                        cat_col += 1;
                    }
                    else if(column_vector < 0)    {
                        cat_col -= 1;
                    }
                }
                else if(row_vector) {
                    if(row_vector > 0)    {
                        cat_row += 1;
                    }
                    else if(row_vector < 0)   {
                        cat_row -= 1;
                    }
                }
                else    {
                    path_progress++;

                }

                update3(cat_col, cat_row);

            }

            counter++;
            int dx=0;
            int dy=0;

            int r = (GPIOC->IDR) & 0xf;
            if (r & 1) { // '4'
                dx -= 1;
            }
            if (r & 2) { // '2'
                dy -= 1;
            }
            if (r & 4) { // '8'
                dy += 1;
            }
            if (r & 8) { // '6'
                dx += 1;
            }
            if (dx !=0 || dy != 0) {
                int update_x = 1;
                int update_y = 1;

                if(mouse_row + dy > 310)    {
                    update_y = 0;       //mouse MAY NOT leave through the entrance
                }
                //REGIONS A - E are hard coded
                if((mouse_col + dx) > 210 || (mouse_col + dx) < 30) {
                    //x is out of bounds (A)
                    update_x = 0;
                }
                if((mouse_col + dx) < 50 && (mouse_col + dx) > 10 && (mouse_row + dy) < 30)   {
                    //if y is out of bounds (B)
                    update_x = 0;
                    update_y = 0;
                }
                else if((mouse_col + dx) > 70 && (mouse_col + dx) < 230 && (mouse_row + dy) < 30)  {
                    //(C)
                    update_x = 0;
                    update_y = 0;
                }
                else if((mouse_col + dx) < 230 && (mouse_col + dx) > 190 && (mouse_row + dy) > 290) {
                    //(D)
                    update_x = 0;
                    update_y = 0;
                }
                else if((mouse_col + dx) < 170 && (mouse_col + dx) > 10 && (mouse_row + dy) > 290) {
                    //(E)
                    update_x = 0;
                    update_y = 0;
                }
                if(!check_wall_new(mouse_row + dy, mouse_col + dx, black_wall_row, black_wall_col))   {
                    update_x = 0;
                    update_y = 0;
                }
                if(update_x)    {
                    mouse_col += dx;
                }
                if(update_y)    {
                    mouse_row += dy;
                }
                update2(mouse_col, mouse_row);
            }

        }
        if(game_over)   {
            LCD_DrawString(90,150, RED, WHITE, "  RETRY  ", 16, 0);
            LCD_DrawString(80,250, RED, WHITE, "BACK TO MAIN", 16, 0);
        }
    }
}

void basic_drawing(void)
{
    LCD_Clear(0);
    LCD_DrawPicture(0,0,&background);
    LCD_DrawString(80,20, BLACK, WHITE, "MOUSE & CAT", 16, 0); // opaque background
    LCD_DrawString(100,150, BLACK, WHITE, "START", 16, 0);
    LCD_DrawString(110,200, BLACK, WHITE, "HELP", 16, 0);
}
void help_content();
void help_content()
{
	LCD_Clear(0);
	LCD_DrawString(70,50, WHITE, BLACK, "INSTRUCTIONS", 16, 0);
	LCD_DrawString(5,80, WHITE, BLACK, "1: Increase your score by getting", 12, 0);
	LCD_DrawString(5,100, WHITE, BLACK, "to the exit each time!.", 12, 0);
	LCD_DrawString(5,120, WHITE, BLACK, "2: The cat will become more", 12, 0);
	LCD_DrawString(5,140, WHITE, BLACK, "INTELLIGENT every time you reach", 12, 0);
	LCD_DrawString(5,160, WHITE, BLACK, "a new maze", 12, 0);
	LCD_DrawString(5,180, WHITE, BLACK, "3: Each time the mouse backs up to the", 12, 0);
	LCD_DrawString(5,200, WHITE, BLACK, "entrance, the cat will camp at the exit", 12, 0);
	LCD_DrawString(5,220, WHITE, BLACK, "area", 12, 0);
	LCD_DrawString(100,250, WHITE, BLACK, "BACK", 16, 0);

}

