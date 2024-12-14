import csv
import matplotlib.pyplot as plt
import math


def main():
    flag_lst=('error', 'point')
    flag=flag_lst[0]

    file_name="a=1.62_b=1.38.csv"
    #file_name="2a=2b=2.852.csv"
    file_name_refer="refer_line_point.csv"

    s_lst=list()
    match_index_lst=list()
    ed_lst=list()
    ed_dot_lst=list()
    efai_lst=list()
    efai_dot_lst=list()
    project_curv_lst=list()
    ego_x_lst=list()
    ego_y_lst=list()
    refer_x_lst=list()
    refer_y_lst=list()
    x_left_lst=list()
    y_left_lst=list()
    x_right_lst=list()
    y_right_lst=list()
    if flag==flag_lst[0]:
        with open(file_name) as f:
            reader=csv.reader(f)
            header_row=next(reader)
            for row in reader:
                s_lst.append(float(row[0]))
                match_index_lst.append(float(row[1]))
                ed_lst.append(float(row[2]))
                ed_dot_lst.append(float(row[3]))
                efai_lst.append(float(row[4])*180/math.pi)
                efai_dot_lst.append(float(row[5])*180/math.pi)
                project_curv_lst.append(float(row[6]))

        print("Read data finished-----")
        # fig=plt.figure()
        # ax1=fig.add_subplot(221)
        # ax1.scatter(s_lst, ed_lst,s=2)
        # plt.xlabel('s (m)')
        # plt.ylabel('ed (m)')

        # ax2=fig.add_subplot(222)
        # ax2.scatter(s_lst, ed_dot_lst,s=2)
        # plt.xlabel('s (m)')
        # plt.ylabel('ed_dot (m)')

        # ax3=fig.add_subplot(223)
        # ax3.scatter(s_lst, efai_lst,s=2)
        # plt.xlabel('s (m)')
        # plt.ylabel('efai (degree)')

        # ax4=fig.add_subplot(224)
        # ax4.scatter(s_lst, efai_dot_lst,s=2)
        # plt.xlabel('s (m)')
        # plt.ylabel('efai_dot (degree)')

        # plt.show()
        #---------------------------
        # plot curvature
        plt.scatter(s_lst, ed_dot_lst,s=2)
        plt.xlabel('s (m)')
        plt.ylabel('road curvature (1/m)')
        plt.show()

    elif flag==flag_lst[1]:
        with open(file_name) as f:
            reader=csv.reader(f)
            header_row=next(reader)
            for row in reader:
                ego_x_lst.append(float(row[7]))
                ego_y_lst.append(float(row[8]))
        with open(file_name_refer) as f:
            reader=csv.reader(f)
            header_row=next(reader)
            for row in reader:
                refer_x_lst.append(float(row[0]))
                refer_y_lst.append(float(row[1]))
                x_left_lst.append(float(row[2]))
                y_left_lst.append(float(row[3]))
                x_right_lst.append(float(row[4]))
                y_right_lst.append(float(row[5]))
        plt.scatter(ego_x_lst,ego_y_lst, s=2,c='lightcoral', label='ego_vehicle_pos')
        plt.scatter(refer_x_lst,refer_y_lst,s=2, c='black', label='reference_line')
        plt.scatter(x_left_lst, y_left_lst, s=2, c='darkgrey', label='left/right_roadedge')
        plt.scatter(x_right_lst, y_right_lst, s=2, c='darkgrey')
        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
        plt.legend(loc='lower right', prop={'size':16})
        plt.title("Ego vehicle trajectory vs. Reference line")
        plt.show()

        

    
if __name__=='__main__':
    main()