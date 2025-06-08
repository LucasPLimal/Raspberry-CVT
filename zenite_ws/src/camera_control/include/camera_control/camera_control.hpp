#include <linux/videodev2.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <iostream>

class InterfaceNode {
public:
    InterfaceNode() {
        fd_ = open("/dev/video0", O_RDWR);
        if (fd_ == -1) {
            perror("Erro ao abrir /dev/video0");
        }
    }

    ~InterfaceNode() {
        if (fd_ != -1) {
            close(fd_);
        }
    }

    
    /// Altera os parâmetros da câmera (use -1 para manter o valor atual)
    void send_params(float brilho, float saturacao, float contraste) {
        if (fd_ == -1) return;

        if (brilho >= 0)
            set_control(V4L2_CID_BRIGHTNESS, brilho * 255);

        if (saturacao >= 0)
            set_control(V4L2_CID_SATURATION, saturacao * 255);

        if (contraste >= 0)
            set_control(V4L2_CID_CONTRAST, contraste * 255);
    }

    /// Leitura do valor atual de um parâmetro
    int get_control(__u32 id) {
        if (fd_ == -1) return -1;

        struct v4l2_control control {};
        control.id = id;
        
        if (ioctl(fd_, VIDIOC_G_CTRL, &control) == -1) {
            perror("Erro ao obter controle");
            return -1;
        }

        return control.value;
    }

private:
    int fd_ = -1;

    /// Aplica um valor a um parâmetro específico
    void set_control(__u32 id, int value) {
        struct v4l2_control control {};
        control.id = id;
        control.value = value;

        if (ioctl(fd_, VIDIOC_S_CTRL, &control) == -1) {
            perror("Erro ao definir controle");
        }
    }
};
