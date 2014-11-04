package pomdp.utilities.datastructures;

import java.io.Serializable;
import java.util.Iterator;
import java.util.Map.Entry;

public abstract class Function implements Serializable {

	private static final long serialVersionUID = 1L;
	
	protected int[] m_aDims;
	protected int m_cDims;
	protected double m_dMinValue;
	protected double m_dMaxValue;
	protected double m_dDefaultValue;
	
	public Function( int[] aDims ){
		int iDim = 0;
		m_cDims = aDims.length;
		m_aDims = new int[m_cDims];
		for( iDim = 0 ; iDim < m_cDims ; iDim++ )
			m_aDims[iDim] = aDims[iDim];
		m_dMinValue = 0.0;
		m_dMaxValue = Double.MAX_VALUE * -1;
		m_dDefaultValue = 0.0;
	}
	
	public void printD2()
	{
		
	}
	
	
	public int getSize( int iDim ){
		return m_aDims[iDim];
	}
	
	public double getMaxValue(){
		return m_dMaxValue;
	}
	
	public double getMinValue(){
		return m_dMinValue;
	}
	
	public abstract double valueAt( int arg1 );
	public abstract double valueAt( int arg1, int arg2 );
	public abstract double valueAt( int arg1, int arg2, int arg3 );
	public abstract void setValue( int arg1, double dValue );
	public abstract void setValue( int arg1, int arg2, double dValue );
	
	/**
	 * ��������Transition Function��һ��ת��
	 * ������iStartState, iActionIdx, iEndState, dValue)
	 * @param arg1
	 * @param arg2
	 * @param arg3
	 * @param dValue
	 */
	public abstract void setValue( int arg1, int arg2, int arg3, double dValue );
	/**
	 *  ��úͿ�ʼ״̬�Ͷ����йصĸ��ʷ�0��ת��
	 * @param arg1
	 * @param arg2
	 * @return
	 */
	public abstract Iterator<Entry<Integer,Double>> getNonZeroEntries( int arg1, int arg2 );
	
	@SuppressWarnings("rawtypes")
	public abstract Iterator getNonZeroEntries();
	
	public abstract int countNonZeroEntries( int arg1, int arg2 );

	protected void setAllValues( int iParam1, int iParam3, double dValue ){
		int iParam2 = 0;
		int cParam2 = m_aDims[1];
		for( iParam2 = 0 ;iParam2 < cParam2 ; iParam2++ ){
			setValue( iParam1, iParam2, iParam3, dValue );
		}
	}
	
	/**
	 * ��������O Function��һ���۲�ֵ����
	 * ���վ�����setValue( iParam1, iParam2, iParam3, dValue )
	 * ���մ���SparseTabularFunction.HashMap<Integer,Double>[][]
	 * ������iAction, iEndState, iObservation, dValue 
	 * 
	 * aDims[0] = m_cActions;
	 * aDims[1] = m_cStates;
	 * aDims[2] = m_cObservations;
	 * 
	 * @param iParam1
	 * @param iParam2
	 * @param iParam3
	 * @param dValue
	 */
	public void setAllValues( int iParam1, int iParam2, int iParam3, double dValue ){
		int cParam1 = m_aDims[0];//m_cActions
		//iAction, iEndState, iObservation��������
		if( iParam1 == -1 && iParam2 == -1 && iParam3 == -1 )
		{
			int cParam2 = m_aDims[1], cParam3 = m_aDims[2]; //m_cStates m_cObservations
			for( iParam1 = 0 ; iParam1 < cParam1 ; iParam1++ )
				for( iParam2 = 0 ; iParam2 < cParam2 ; iParam2++ )
					for( iParam3 = 0 ; iParam3 < cParam3 ; iParam3++ )
						setValue( iParam1, iParam2, iParam3, dValue );
			return;
		
		}
		//iAction����
		if( iParam1 == -1 ){
			for( iParam1 = 0 ; iParam1 < cParam1 ; iParam1++ ){
				//iEndState����
				if( iParam2 == -1 ){
					setAllValues( iParam1, iParam3, dValue );
				}
				else{
					setValue( iParam1, iParam2, iParam3, dValue );
				}
			}
		}
		else{
			//iEndState����
			if( iParam2 == -1 ){
				setAllValues( iParam1, iParam3, dValue );
			}
			else{
				setValue( iParam1, iParam2, iParam3, dValue );
			}
		}
	}
	
	public void setValue( int[] parameters, double dValue ){
		switch( parameters.length ){
		case 1:
			setValue( parameters[0], dValue );
		case 2:
			setValue( parameters[0], parameters[1], dValue  );
		case 3:
			setValue( parameters[0], parameters[1], parameters[2], dValue );
		}
	}
	
	public double valueAt( int[] parameters ){
		switch( parameters.length ){
			case 0:
				return m_dDefaultValue;
			case 1:
				return valueAt( parameters[0] );
			case 2:
				return valueAt( parameters[0], parameters[1]  );
			case 3:
				return valueAt( parameters[0], parameters[1], parameters[2] );
		}
		return 0.0;
	}
	
	public abstract int countEntries();
	public abstract int countNonZeroEntries();

}
