    function varargout = prueba2(varargin)
    % PRUEBA2 MATLAB code for prueba2.fig
    %      PRUEBA2, by itself, creates a new PRUEBA2 or raises the existing
    %      singleton*.
    %
    %      H = PRUEBA2 returns the handle to a new PRUEBA2 or the handle to
    %      the existing singleton*.
    %
    %      PRUEBA2('CALLBACK',hObject,eventData,handles,...) calls the local
    %      function named CALLBACK in PRUEBA2.M with the given input arguments.
    %d 
    %      PRUEBA2('Property','Value',...) creates a new PRUEBA2 or raises the
    %      existing singleton*.  Starting from the left, property value pairs are
    %      applied to the GUI before prueba2_OpeningFcn gets called.  An
    %      unrecognized property name or invalid value makes property application
    %      stop.  All inputs are passed to prueba2_OpeningFcn via varargin.
    %
    %      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
    %      instance to run (singleton)".
    %
    % See also: GUIDE, GUIDATA, GUIHANDLES

    % Edit the above text to modify the response to help prueba2

    % Last Modified by GUIDE v2.5 22-Jun-2023 23:55:03

    % Begin initialization code - DO NOT EDIT
    gui_Singleton = 1;
    gui_State = struct('gui_Name',       mfilename, ...
                       'gui_Singleton',  gui_Singleton, ...
                       'gui_OpeningFcn', @prueba2_OpeningFcn, ...
                       'gui_OutputFcn',  @prueba2_OutputFcn, ...
                       'gui_LayoutFcn',  [] , ...
                       'gui_Callback',   []);
    if nargin && ischar(varargin{1})
        gui_State.gui_Callback = str2func(varargin{1});
    end

    if nargout
        [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
    else
        gui_mainfcn(gui_State, varargin{:});
    end
    % End initialization code - DO NOT EDIT


    % --- Executes just before prueba2 is made visible.
    function prueba2_OpeningFcn(hObject, eventdata, handles, varargin)
    % This function has no output args, see OutputFcn.
    % hObject    handle to figure
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    % varargin   command line arguments to prueba2 (see VARARGIN)

    handles.a=arduino; %crear un objeto para el arduino
    set(handles.Puerto,'string',handles.a.Port);    %cuadro de texto puerto
    set(handles.Tarjeta,'string',handles.a.Board);  %cuadro de texto placa arduino
    handles.output = hObject;   %objeto asociado1
    guidata(hObject, handles);  %objeto asociado2

    axes(handles.axes3); 
    [x,map]=imread('CAPIBARA-FONDO.png'); 
    image(x); 
    colormap(map); 
    axis off
    hold on
    % UIWAIT makes prueba2 wait for user response (see UIRESUME)
    % uiwait(handles.figure1);


    % --- Outputs from this function are returned to the command line.
    function varargout = prueba2_OutputFcn(hObject, eventdata, handles) 
    % varargout  cell array for returning output args (see VARARGOUT);
    % hObject    handle to figure
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Get default command line output from handles structure
    varargout{1} = handles.output;


    % --- Executes when user attempts to close figure1.
    function figure1_CloseRequestFcn(hObject, eventdata, handles)
    % hObject    handle to figure1 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hint: delete(hObject) closes the figure
    clear handles.a
    delete(hObject);


    % --- Executes on button press in LED.
    function LED_Callback(hObject, eventdata, handles)
    % hObject    handle to LED (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hint: get(hObject,'Value') returns toggle state of LED
    v=get(hObject,'value');
    if v==1
    writeDigitalPin(handles.a,'D13',1);
    set(hObject,'BackgroundColor',[0 0.5 0.04],'ForegroundColor',[0 1 0]);
    else
    writeDigitalPin(handles.a,'D13',0);   
    set(hObject,'BackgroundColor',[0.94 0.94 0.94],'ForegroundColor',[0 0 0]);
    end


    % --- Executes on button press in Muestreo.
    function Muestreo_Callback(hObject, eventdata, handles)
    axes(handles.axes1);
    cla;

    k = 1;
    vmu = get(hObject, 'Value');
    while vmu == 1
        muestra(k) = k;
        voltaje(k) = readVoltage(handles.a, 'A0');
        temperatura(k) = (voltaje(k)/10) * 1000; % Conversión de voltaje a temperatura en grados Celsius
        plot(muestra, temperatura);
        xlabel('Tiempo');
        ylabel('Temperatura (°C)');
        grid minor;
        drawnow;
        k = k + 1;
        set(hObject, 'BackgroundColor', [0 0 1], 'ForegroundColor', [0 1 0], 'String', 'Tomando muestras');
        pause(1);

        % Actualizar el valor de vmu
        vmu = get(hObject, 'Value');
    end


    % --- Executes during object creation, after setting all properties.
    function axes3_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to axes3 (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: place code in OpeningFcn to populate axes3


    % --- Executes on button press in PID.
    function PID_Callback(hObject, eventdata, handles)
    % hObject    handle to PID (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    % Imprimir el valor del setpoint en la ventana de comandos
    % Obtener el valor de setpoint de handles


    % Obtener el valor de setpoint de handles
    setpoint = handles.setpoint

    % Configurar el gráfico
    axes(handles.axes1);
    hLine = plot(NaN, NaN, 'b-'); % Gráfico para el valor del sensor (en azul)
    hold on;
    hLineSetpoint = plot(NaN, NaN, 'r'); % Gráfico para el setpoint (en rojo)
    hLineControl = plot(NaN, NaN, 'g'); % Gráfico para el control (en verde)
    hold off;
    xlabel('Tiempo');
    ylabel('Valor');
    grid on;
    legend('Sensor', 'Setpoint');

    %Parametros iniciales
    N = 600;           % Numero de muestras
    Fs = 10;             % Frecuencia de muestreo, Hz
    Ts = 1/Fs          % Periodo de muestreo [seg]
    t = 0:Ts:N*Ts;      % Vector de tiempo de simulacion [seg]
    kini = 3;
    Stop = 1;           

    umax = 5;
    umin = 0;
    %senales
    yr = [setpoint*ones(1,N)];
    l = ones(N,1);
    y = zeros(N,1);
    e = zeros(N,1);
    Te = zeros(N,1);
    % Algoritmo

    %Modelo de la planta de primer orden
    Kp = 0.16;                    %Ganancia de la planta
    theta = 1;                  %atraso continuo
    taula = 7;                  %Tau
    num = Kp;           
    den = [taula 1];
    Gs = tf(num,den)

    % Ziegler-Nichols
    kp = (1.2*taula/(theta))-5
    Ti = (2*theta/(taula))
    Td = (theta/2)-0.4

    %otro
    q0 = kp*(1+Td/Ts)
    q1 = -kp*(1+2*Td/Ts-Ts/Ti)
    q2 = kp*Td/Ts

    for k=kini:N
        Tc = cputime;

        % Salida del proceso
        valor_sensor = readVoltage(handles.a, 'A0');
        y(k) = (valor_sensor / 10) * 1000;
        %error
        e(k) = yr(k)- y(k);

        %senal de control
        l(k) = l(k-1)+q0*e(k)+q1*e(k-1)+q2*e(k-2);
        l(k)=l(k)/20;

        % Saturacion de la señal de control
        if l(k) >= umax
            l(k) = umax;
        elseif l(k) <= umin
            l(k)=umin;
        end
        writePWMVoltage(handles.a,'D3',l(k));
    %---------------------graficas---------------------
        % Obtener los datos actuales del gráfico
        xdata = get(hLine, 'XData');
        ydata_sensor = get(hLine, 'YData');
        ydata_setpoint = get(hLineSetpoint, 'YData');
        ydata_control = get(hLineControl, 'YData');

        % Actualizar los datos del gráfico
        xdata = [xdata, k];
        ydata_sensor = [ydata_sensor, y(k)];
        ydata_setpoint = [ydata_setpoint, yr(k)];
        ydata_control = [ydata_control, l(k)*20];

        set(hLine, 'XData', xdata, 'YData', ydata_sensor);
        set(hLineSetpoint, 'XData', xdata, 'YData', ydata_setpoint);
        set(hLineControl, 'XData', xdata, 'YData', ydata_control);

        % Ajustar el límite del eje x
        xlim([0, N]);

        % Actualizar la visualización del gráfico
        drawnow;
    %------------garantizando tiempo de muestreo------------
        Te(k) = Ts-(cputime-Tc); 
        if Te(k) > 0
            pause(Te(k));
        end

        if Stop == 0
            writePWMVoltage(ar,'D3',0);
            break
        end
    end



    function SETPOINT_Callback(hObject, eventdata, handles)
    % hObject    handle to SETPOINT (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Obtener el valor ingresado en el cuadro de texto SETPOINT
    setpoint = str2double(get(hObject, 'String'));

    % Actualizar la estructura handles con el valor de setpoint
    handles.setpoint = setpoint;

    % Guardar los cambios en la estructura handles
    guidata(hObject, handles);






    % --- Executes during object creation, after setting all properties.
    function SETPOINT_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to SETPOINT (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: edit controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end


    % --- Executes on button press in deadbeat.
    function deadbeat_Callback(hObject, eventdata, handles)
    % hObject    handle to deadbeat (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    % Obtener el valor de setpoint de handles
    setpoint = handles.setpoint;

    % Configurar el gráfico
    axes(handles.axes1);
    hLine = plot(NaN, NaN, 'b-'); % Gráfico para el valor del sensor (en azul)
    hold on;
    hLineSetpoint = plot(NaN, NaN, 'r'); % Gráfico para el setpoint (en rojo)
    hLineControl = plot(NaN, NaN, 'g'); % Gráfico para el control (en verde)
    hold off;
    xlabel('Tiempo');
    ylabel('Valor');
    grid on;
    legend('Sensor', 'Setpoint');

    %Parametros iniciales
    N = 600;           % Numero de muestras
    Fs = 10;             % Frecuencia de muestreo, Hz
    Ts = 1/Fs;          % Periodo de muestreo [seg]
    t = 0:Ts:N*Ts;      % Vector de tiempo de simulacion [seg]
    kini = 4;
    Stop = 1;           

    umax = 5;
    umin = 0;
    %senales
    yr = [setpoint*ones(1,N)];
    u = ones(N,1);
    y = zeros(N,1);
    e = zeros(N,1);
    Te = zeros(N,1);
    % Algoritmo

    %modelo que creo que es el correcto pero no me da
    Kp = 0.16;                 % Ganancia del proceso
    theta = 1;
    taula = 7;             % Constante de tiempo de lazo abierto
    num = Kp;
    den = [taula 1];
    Gs = tf(num,den)

    % Modelo discreto del proceso
    Gpz = c2d(Gs,Ts,'zoh');
    Gpz.Variable='z^-1';
    d = 2;
    B = Gpz.num{:};
    A = Gpz.den{:};
    b0 = B(2);
    a1 = A(2);

    % Expresion simplificada
    % u1(k) = u(k-d)+p0*e(k)+p1*e(k-1); 
    q = exp(-d/taula)
    p0 = 1/(Kp*(1-q))-24.6
    p1 = -q/(Kp*(1-q))+18.7

    for k=kini:N
        Tc = cputime;

        % Salida del proceso
        valor_sensor = readVoltage(handles.a, 'A0');
        y(k) = (valor_sensor / 10) * 1000;

        %error
        e(k) = yr(k)-y(k);

        %senal de control
        u(k) = u(k-(d+1))+p0*e(k)+p1*e(k-1);

        % Saturacion de la señal de control
        if u(k) >= umax
            u(k) = umax;
        elseif u(k) <= umin
            u(k)=umin;
        end
        writePWMVoltage(handles.a,'D3',u(k));
    %---------------------graficas---------------------
        xdata = get(hLine, 'XData');
        ydata_sensor = get(hLine, 'YData');
        ydata_setpoint = get(hLineSetpoint, 'YData');
        ydata_control = get(hLineControl, 'YData');

        % Actualizar los datos del gráfico
        xdata = [xdata, k];
        ydata_sensor = [ydata_sensor, y(k)];
        ydata_setpoint = [ydata_setpoint, yr(k)];
        ydata_control = [ydata_control, u(k)*20];

        set(hLine, 'XData', xdata, 'YData', ydata_sensor);
        set(hLineSetpoint, 'XData', xdata, 'YData', ydata_setpoint);
        set(hLineControl, 'XData', xdata, 'YData', ydata_control);

        % Ajustar el límite del eje x
        xlim([0, N]);

        % Actualizar la visualización del gráfico
        drawnow;
    %------------garantizando tiempo de muestreo------------
        Te(k) = Ts-(cputime-Tc); 
        if Te(k) > 0
            pause(Te(k));
        end

        if Stop == 0
            writePWMVoltage(handles.a,'D3',0);
            break
        end
    end
    writePWMVoltage(ar,'D3',0);


    % --- Executes on button press in lazoabierto.
    function lazoabierto_Callback(hObject, eventdata, handles)
    % hObject    handle to lazoabierto (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    %Parametros iniciales
    N = 300;           % Numero de muestras
    Fs = 10;             % Frecuencia de muestreo, Hz
    Ts = 1/Fs;          % Periodo de muestreo [seg]
    t = 0:Ts:N*Ts;      % Vector de tiempo de simulacion [seg]
    kini = 1;
    Stop = 1;    

    % Configurar el gráfico
    axes(handles.axes1);
    hLine = plot(NaN, NaN, 'b-'); % Gráfico para el valor del sensor (en azul)
    hold on;
    hLineInput = plot(NaN, NaN, 'g'); % Gráfico para el control (en verde)
    hold off;
    xlabel('Tiempo');
    ylabel('Valor');
    grid on;
    legend('Sensor', 'Input');

    umax = 5;
    umin = 0;
    %senales
    y = zeros(N,1);
    u = 3.4*ones(N,1);
    Te = zeros(N,1);
    % Algoritmo
    for k=kini:N
        Tc = cputime;

        % Salida del proceso
        valor_sensor = readVoltage(handles.a, 'A0');
        temperatura = (valor_sensor / 10) * 1000;

        % Saturacion de la señal de control
        if u(k) >= umax
            u(k) = umax;
        elseif u(k) <= umin
            u(k)=umin;
        end
        writePWMVoltage(handles.a,'D3',u(k));
    %---------------------graficas---------------------
        xdata = get(hLine, 'XData');
        ydata_sensor = get(hLine, 'YData');
        ydata_Input = get(hLineInput, 'YData');

        % Actualizar los datos del gráfico
        xdata = [xdata, k];
        ydata_sensor = [ydata_sensor, temperatura];
        ydata_Input = [ydata_Input, u(k)*20];

        set(hLine, 'XData', xdata, 'YData', ydata_sensor);
        set(hLineInput, 'XData', xdata, 'YData', ydata_Input);

        % Ajustar el límite del eje x
        xlim([0, N]);

        % Actualizar la visualización del gráfico
        drawnow;
    %------------garantizando tiempo de muestreo------------
        Te(k) = Ts-(cputime-Tc); 
        if Te(k) > 0
            pause(Te(k));
        end

        if Stop == 0
            writePWMVoltage(handles.a,'D3',0);
            break
        end
    end
    writePWMVoltage(handles.a,'D3',0);
