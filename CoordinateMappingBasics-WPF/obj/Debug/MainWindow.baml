   M S B A M L   `   `   `  ����  CoordinateMappingBasics-WPFA    ;Microsoft.Samples.Kinect.CoordinateMappingBasics.MainWindow   5   	   R NWindowsBase, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35W SPresentationCore, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35\ XPresentationFramework, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35D 9http://schemas.microsoft.com/winfx/2006/xaml/presentation    5   	   R NSystem.Xaml, Version=4.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089:x,http://schemas.microsoft.com/winfx/2006/xaml     5   	   -   5          � Title$"  2d-correspondances generator��5   	   $	��600��5   	   $	��700��6    � 	Resources 5         MediumGreyBrush �� Color  KinectPurpleBrush  KinectBlueBrush �� 
TargetType �� Property �� Value  ScreenshotButton �� 
TargetType �� Orientation ��Name	 �� TextAlignment
 ��Name �� Triggers P� Property P� Value �� 
TargetName��%a  &	        &	 #     &	 F     '�� i     &	 �     7  7  �� 5   
   $ 	#ff6e6e6e��62   �� 5	   
   $ 	#ff52318f��64   �� 5
   
   $ 	#ff00BCF2��62   �� 5   
   " ��6   .�5      ��  �@J�SnapsToDevicePixels . 64   5      �� 5   
   " ��6   .�5      ��  � �� 5      �� 5      " ��6&   .R�5      � .+�5      �� $ 
Horizontal=�6)   
������ .+�5   "   �� $ ScreenshotText��6,   #��z��6D   $
	 Left=�6e   $��Center=�6{   8��  $��15(�6�   � 5   "   $��9,0,0,0q�6'   .+�5   &   �� $
 ScreenshotNormal��6,   $!��Images\ScreenshotNormal.png��6F   $
��None=�6k   $��Center=�6z   �� 5   &   $
 ScreenshotHover��6,   $ ��Images\ScreenshotHover.png��6E   $
��None=�6i   $��Center=�6x   $y�	Collapsed=�6�   5   #   5      5       5      P� 5       �@J�IsMouseOver . . �5   "   �� $ ScreenshotNormal��6Q    � y�$ 	Collapsed=�6?   6Q   �� 5   "   $ ScreenshotHover��6O    � y�$ Visible=�6?   6O   �� 5   "   8  $ ScreenshotText��6h    � ��5       5"      5$      5%      5&      .��5'      � $��	10 0 10 0q�6    � RowDefinitions 5(   
   �� 5)      $
��Auto �6   �� 5*      $��* �6   �� 5+      $
��Auto �6   �� 5,      $
��Auto �6   .+�5/   
   (� $��1��6   $��Center=�6   .��50      � -   5        $��CompositeImage��50      .+�51      �� $��Images\Background.png��6   $��UniformToFill=�67   �� 52      $��UniformToFill=�67   $��-29,-10,0,0q�6O   ��6   �� *ImageSource+6O   53      54      �� 56   
   -   5        $��2��56      #��[ 6   $��
Screenshot��6H   $
��Auto��6]   $��Right=�6k   $��Center=�6�   $��
10 10 0 10q�6�   6�   �� 58   
   -   5        $��	statusBar��58   ?   $��3��6   $��Stretch=�6!   $��Bottom=�6P   
�������#��[  6~   .@�59      �� ��6   �� *
StatusText+5:      �� 5;   
   -   5        $��projectButton��5;   #   $��Project��6   $
��Left=�6T   $��279,9.8,0,0q�6o   $��2��6�   $	��Top=�6�   $��75��6�   �� 5<   
   -   5        $��calculateButton��5<   %   $��	Calculate��6   $
��Left=�6Z   $��359,9.8,0,0q�6u   $��2��6�   $	��Top=�6�   $��75��6�   �� 5=   
   -   5        $��captureDepthButton��5=   )   $��Capture Depth��6   $
��Left=�6d   $��105,10.8,0,23q�6   $��2��6�   $��89��6�    J� RenderTransformOrigin$ 0.225,0.573&�6�   �� 5>   
   -   5        $��filterDepthButton��5>   (   $��Filter Depth��6   $
��Left=�6a   $��199,10.8,0,0q�6|   $��2��6�   $	��Top=�6�   $��75��6�   �� 5?   
   -	   5        $��	runButton��5?      $	��Run��6   $
��Left=�6M   $��10,10.8,0,0q�6h   $��2��6}   $	��Top=�6�   $��75��6�   5A      5B      