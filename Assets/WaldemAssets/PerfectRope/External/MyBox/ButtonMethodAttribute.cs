// MIT License
// Copyright (c) 2018 Andrew Rumak
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
// THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

using System;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;
using System.Reflection;
using UnityEditor;
using System.Text.RegularExpressions;
using Object = UnityEngine.Object;

namespace External.MyBox
{
	[AttributeUsage(AttributeTargets.Method)]
	public class ButtonMethodAttribute : Attribute
	{
		public readonly ButtonMethodDrawOrder DrawOrder;

		public ButtonMethodAttribute(ButtonMethodDrawOrder drawOrder = ButtonMethodDrawOrder.AfterInspector)
		{
			DrawOrder = drawOrder;
		}
	}

	public enum ButtonMethodDrawOrder
	{
		BeforeInspector, 
		AfterInspector
	}
#if UNITY_EDITOR

	[CustomEditor(typeof(Object), true), CanEditMultipleObjects]
    public class UnityObjectEditor : Editor
    {
        private FoldoutAttributeHandler _foldout;
        private ButtonMethodHandler _buttonMethod; 
        
        private void OnEnable()
        {
            if (target == null) return;
            
            _foldout = new FoldoutAttributeHandler(target, serializedObject);
            _buttonMethod = new ButtonMethodHandler(target);
        }

        private void OnDisable()
        {
            _foldout?.OnDisable();
        }

        public override void OnInspectorGUI()
        {
            _buttonMethod?.OnBeforeInspectorGUI();
            
            if (_foldout != null)
            {
                _foldout.Update();
                if (!_foldout.OverrideInspector) base.OnInspectorGUI();
                else _foldout.OnInspectorGUI();
            }

            _buttonMethod?.OnAfterInspectorGUI();
        }
    }

	public class ButtonMethodHandler
	{
		public readonly List<(MethodInfo Method, string Name, ButtonMethodDrawOrder order)> TargetMethods;
		public int Amount => TargetMethods?.Count ?? 0;
		
		private readonly Object _target;

		public ButtonMethodHandler(Object target)
		{
			_target = target;
			
			var type = target.GetType();
			var bindings = BindingFlags.Instance | BindingFlags.Static | BindingFlags.Public | BindingFlags.NonPublic;
			var members = type.GetMembers(bindings).Where(IsButtonMethod);

			foreach (var member in members)
			{
				var method = member as MethodInfo;
				if (method == null) continue;
				
				if (IsValidMember(method, member))
				{
					var attribute = (ButtonMethodAttribute)Attribute.GetCustomAttribute(method, typeof(ButtonMethodAttribute));
					if (TargetMethods == null) TargetMethods = new List<(MethodInfo, string, ButtonMethodDrawOrder)>();
					TargetMethods.Add((method, SplitCamelCase(method.Name), attribute.DrawOrder));
				}
			}
		}
		
		public string SplitCamelCase(string camelCaseString)
		{
			if (string.IsNullOrEmpty(camelCaseString)) return camelCaseString;

			string camelCase = Regex.Replace(Regex.Replace(camelCaseString, @"(\P{Ll})(\P{Ll}\p{Ll})", "$1 $2"), @"(\p{Ll})(\P{Ll})", "$1 $2");
			string firstLetter = camelCase.Substring(0, 1).ToUpper();

			if (camelCaseString.Length > 1)
			{
				string rest = camelCase.Substring(1);

				return firstLetter + rest;
			}

			return firstLetter;
		}

		public void OnBeforeInspectorGUI()
		{
			if (TargetMethods == null) return;

			foreach (var method in TargetMethods)
			{
				if (method.order != ButtonMethodDrawOrder.BeforeInspector) continue;
				
				if (GUILayout.Button(method.Name)) InvokeMethod(_target, method.Method);
			}
			
			EditorGUILayout.Space();
		}

		public void OnAfterInspectorGUI()
		{
			if (TargetMethods == null) return;
			EditorGUILayout.Space();

			foreach (var method in TargetMethods)
			{
				if (method.order != ButtonMethodDrawOrder.AfterInspector) continue;
				
				if (GUILayout.Button(method.Name)) InvokeMethod(_target, method.Method);
			}
		}
		
		public void Invoke(MethodInfo method) => InvokeMethod(_target, method);

		
		private void InvokeMethod(Object target, MethodInfo method)
		{
			var result = method.Invoke(target, null);

			if (result != null)
			{
				var message = $"{result} \nResult of Method '{method.Name}' invocation on object {target.name}";
				Debug.Log(message, target);
			}
		}
		
		private bool IsButtonMethod(MemberInfo memberInfo)
		{
			return Attribute.IsDefined(memberInfo, typeof(ButtonMethodAttribute));
		}
			
		private bool IsValidMember(MethodInfo method, MemberInfo member)
		{
			if (method == null)
			{
				Debug.LogWarning(
					$"Property <color=brown>{member.Name}</color>.Reason: Member is not a method but has EditorButtonAttribute!");
				return false;
			}

			if (method.GetParameters().Length > 0)
			{
				Debug.LogWarning(
					$"Method <color=brown>{method.Name}</color>.Reason: Methods with parameters is not supported by EditorButtonAttribute!");
				return false;
			}

			return true;
		}
	}
#endif
}
